"""Sparkplug B bridge for the DepthAI camera node.

Standalone Python script — no ROS dependency. Drives CameraDetector directly
and bridges detection results to MQTT using Sparkplug B (ISA-95).

Mirrors the structure of agx_arm_gui/agx_arm_gui/spb_bridge_node.py.

ISA-95 identity: GID=DMATDTS_DLSU_LS_MiniFactory, Node=camera_node, Device=depthai_camera
Full DBIRTH:     spBv1.0/DMATDTS_DLSU_LS_MiniFactory/DBIRTH/camera_node/depthai_camera

Usage:
    python camera_spb_node.py
    CAMERA_NODE_CONFIG=/path/to/camera_params.yaml python camera_spb_node.py
"""

import json
import queue
import ssl
import threading
import time
from typing import Callable, Optional

import paho.mqtt.client as mqtt

import tahu.sparkplug_b_pb2 as spb_pb2
from tahu.sparkplug_b import (
    MetricDataType,
    addMetric,
    getNodeDeathPayload,
    getNodeBirthPayload,
    getDeviceBirthPayload,
    getDdataPayload,
)

from camera import CameraDetector
from config_loader import load_config

SPB_NAMESPACE = "spBv1.0"

# ---------------------------------------------------------------------------
# One-hot Boolean state names — Status/State/Current
# ---------------------------------------------------------------------------
STATE_IDLE     = "Idle"
STATE_EXECUTE  = "Execute"
STATE_COMPLETE = "Complete"
STATE_ABORTED  = "Aborted"

_STATE_BOOL_TAGS = {
    STATE_IDLE:     "Status/State/Current/Idle",
    STATE_EXECUTE:  "Status/State/Current/Execute",
    STATE_COMPLETE: "Status/State/Current/Complete",
    STATE_ABORTED:  "Status/State/Current/Aborted",
}

# ---------------------------------------------------------------------------
# Cmd/CntrlCmd Boolean sub-tag names — write True to trigger
# ---------------------------------------------------------------------------
CMD_RESET = "Reset"
CMD_START = "Start"
CMD_STOP  = "Stop"
CMD_CLEAR = "Clear"

_CMD_BOOL_TAGS = {
    CMD_RESET: "Cmd/CntrlCmd/Reset",
    CMD_START: "Cmd/CntrlCmd/Start",
    CMD_STOP:  "Cmd/CntrlCmd/Stop",
    CMD_CLEAR: "Cmd/CntrlCmd/Clear",
}

# ---------------------------------------------------------------------------
# Alarm definitions — code → (priority, message)
# Priority: 1=Critical, 2=High, 4=Maintenance (NAMUR NE107 mapping)
# ---------------------------------------------------------------------------
ALARM_NORMAL = 1
ALARM_UNACK  = 2

ALARM_DEFINITIONS = {
    8001: (1, "CameraOffline"),
    8002: (2, "DetectionTimeout"),
    8003: (1, "PrimaryHostOffline"),
}


# ---------------------------------------------------------------------------
# Bridge
# ---------------------------------------------------------------------------

class CameraSpbBridge:
    """Sparkplug B bridge that drives CameraDetector and publishes results."""

    def __init__(
        self,
        frame_provider: Optional[Callable[[], tuple[bool, int, int]]] = None,
        broker_type: Optional[str] = None,
    ):
        """
        frame_provider: optional callable () -> (pass_, pellet_px, foreign_px).
        When supplied (GUI mode), the bridge pulls detection results from the
        already-running VideoWorker pipeline instead of opening DepthAI itself.
        When None (headless mode), the bridge opens DepthAI directly per cycle.
        May raise queue.Empty if no frame arrives within the detection timeout.

        broker_type: "local" | "hivemq" — overrides camera_params.yaml when set.
        """
        cfg = load_config()
        if broker_type == "hivemq":
            broker = cfg.hivemq_broker
        elif broker_type == "local":
            broker = cfg.local_broker
        else:
            broker = cfg.active_broker()
        self._frame_provider = frame_provider

        self._prefix           = cfg.metric_prefix
        self._group_id         = cfg.spb_group_id
        self._edge_node_id     = cfg.spb_edge_node_id
        self._device_id        = cfg.spb_device_id
        self._primary_host_id  = cfg.primary_host_id
        self._heartbeat_interval = cfg.heartbeat_interval_s
        self._detection_timeout  = cfg.detection_timeout_s
        self._cam_cfg = cfg

        # Reverse lookup: full metric name → CMD code (built after _m is available)
        self._cmd_bool_names: dict = {
            self._m(suffix): code for code, suffix in _CMD_BOOL_TAGS.items()
        }

        # ── Sparkplug topics ─────────────────────────────────────────────
        ns, g = SPB_NAMESPACE, self._group_id
        n,  d = self._edge_node_id, self._device_id
        self._NBIRTH_TOPIC = f"{ns}/{g}/NBIRTH/{n}"
        self._NDEATH_TOPIC = f"{ns}/{g}/NDEATH/{n}"
        self._DBIRTH_TOPIC = f"{ns}/{g}/DBIRTH/{n}/{d}"
        self._DDATA_TOPIC  = f"{ns}/{g}/DDATA/{n}/{d}"
        self._DCMD_TOPIC   = f"{ns}/{g}/DCMD/{n}/{d}"
        self._STATE_TOPIC  = f"{ns}/STATE/{self._primary_host_id}"

        # ── Runtime state ─────────────────────────────────────────────────
        self._state: str     = STATE_IDLE
        self._heartbeat      = False
        self._connected      = False
        self._primary_seen   = False
        self._primary_online = False
        self._in_flight      = False
        self._stop_event     = threading.Event()

        self._alarm_states: dict = {c: ALARM_NORMAL for c in ALARM_DEFINITIONS}
        self._alarm_onsets: dict = {c: 0            for c in ALARM_DEFINITIONS}
        self._last_published: dict = {}
        self._last_result: tuple[bool, int, int] | None = None  # (pass_, pellet_px, foreign_px)

        # ── MQTT client ───────────────────────────────────────────────────
        # Use a distinct client_id from the arm bridge (which uses edge_node_id alone)
        # so both can connect to the same broker without kicking each other off.
        client_id = f"{self._edge_node_id}_camera_node"
        self._mqtt = mqtt.Client(
            callback_api_version=mqtt.CallbackAPIVersion.VERSION1,
            client_id=client_id,
            protocol=mqtt.MQTTv311,
        )
        if broker.use_tls:
            self._mqtt.tls_set(
                cert_reqs=ssl.CERT_REQUIRED,
                tls_version=ssl.PROTOCOL_TLS_CLIENT,
            )
        if broker.username:
            self._mqtt.username_pw_set(broker.username, broker.password)

        self._mqtt.on_connect    = self._on_mqtt_connect
        self._mqtt.on_disconnect = self._on_mqtt_disconnect
        self._mqtt.on_message    = self._on_mqtt_message

        # LWT registered before connect so broker publishes NDEATH on our behalf
        lwt = getNodeDeathPayload()
        self._mqtt.will_set(
            self._NDEATH_TOPIC,
            lwt.SerializeToString(),
            qos=1,
            retain=False,
        )

        self._mqtt.connect_async(broker.host, broker.port, keepalive=60)
        self._mqtt.loop_start()

        print(
            f"[SPB] Camera bridge starting — "
            f"{self._group_id}/{self._edge_node_id}/{self._device_id} | "
            f"prefix={self._prefix} | "
            f"broker={broker.host}:{broker.port} "
            f"(TLS={'on' if broker.use_tls else 'off'})"
        )

    # ------------------------------------------------------------------
    # Metric prefix helper
    # ------------------------------------------------------------------

    def _m(self, suffix: str) -> str:
        return f"{self._prefix}/{suffix}" if self._prefix else suffix

    # ------------------------------------------------------------------
    # MQTT callbacks
    # ------------------------------------------------------------------

    def _on_mqtt_connect(self, client, userdata, flags, rc):
        if rc != 0:
            print(f"[SPB] Broker rejected connection, rc={rc}")
            return
        print("[SPB] MQTT connected — publishing NBIRTH / DBIRTH")
        self._connected = True
        self._last_published.clear()
        client.subscribe(self._DCMD_TOPIC, qos=1)
        client.subscribe(self._STATE_TOPIC, qos=1)
        self._publish_nbirth(client)
        self._publish_dbirth(client)

    def _on_mqtt_disconnect(self, client, userdata, rc):
        self._connected = False
        print(f"[SPB] MQTT disconnected (rc={rc}), paho will reconnect")

    def _on_mqtt_message(self, client, userdata, msg):
        if msg.topic == self._DCMD_TOPIC:
            self._handle_dcmd(msg.payload)
        elif msg.topic == self._STATE_TOPIC:
            self._handle_primary_state(msg.payload)

    # ------------------------------------------------------------------
    # Birth publishing
    # ------------------------------------------------------------------

    def _publish_nbirth(self, client):
        payload = getNodeBirthPayload()
        addMetric(payload, "Node Control/Rebirth", None, MetricDataType.Boolean, False)
        client.publish(self._NBIRTH_TOPIC, payload.SerializeToString(), qos=1)

    def _publish_dbirth(self, client):
        payload = getDeviceBirthPayload()

        # State one-hot Booleans
        for state_code, suffix in _STATE_BOOL_TAGS.items():
            addMetric(payload, self._m(suffix), None,
                      MetricDataType.Boolean, self._state == state_code)
        addMetric(payload, self._m("Status/Heartbeat"), None,
                  MetricDataType.Boolean, self._heartbeat)

        # Cmd Boolean sub-tags
        for suffix in _CMD_BOOL_TAGS.values():
            addMetric(payload, self._m(suffix), None, MetricDataType.Boolean, False)

        # Detection result (initial zeros / false)
        addMetric(payload, self._m("Result/Last/Pass"),              None, MetricDataType.Boolean, False)
        addMetric(payload, self._m("Result/Last/PelletCount"),       None, MetricDataType.Int32,   0)
        addMetric(payload, self._m("Result/Last/PelletPixelCount"),  None, MetricDataType.Int32,   0)
        addMetric(payload, self._m("Result/Last/ForeignPixelCount"), None, MetricDataType.Int32,   0)
        addMetric(payload, self._m("Result/Last/TimestampMs"),       None, MetricDataType.Int64,   0)

        # Alarm tree — all codes declared at Normal so SCADA gets full catalogue at DBIRTH
        for code, (priority, message) in ALARM_DEFINITIONS.items():
            addMetric(payload, self._m(f"Alarm/Active/{code}/State"),    None,
                      MetricDataType.Int32,  self._alarm_states[code])
            addMetric(payload, self._m(f"Alarm/Active/{code}/Priority"), None,
                      MetricDataType.Int32,  priority)
            addMetric(payload, self._m(f"Alarm/Active/{code}/Message"),  None,
                      MetricDataType.String, message)
            addMetric(payload, self._m(f"Alarm/Active/{code}/OnsetMs"),  None,
                      MetricDataType.Int64,  self._alarm_onsets[code])
        addMetric(payload, self._m("Alarm/Summary/ActiveCount"), None,
                  MetricDataType.Int32, self._active_alarm_count())

        client.publish(self._DBIRTH_TOPIC, payload.SerializeToString(), qos=1)

    # ------------------------------------------------------------------
    # DDATA helper
    # ------------------------------------------------------------------

    def _publish_ddata(self, metrics: dict):
        """Publish metrics unconditionally. metrics: {name: (MetricDataType, value)}"""
        if not self._connected or not metrics:
            return
        payload = getDdataPayload()
        for name, (dtype, value) in metrics.items():
            addMetric(payload, name, None, dtype, value)
            self._last_published[name] = value
        self._mqtt.publish(self._DDATA_TOPIC, payload.SerializeToString(), qos=0)

    # ------------------------------------------------------------------
    # DCMD handling
    # ------------------------------------------------------------------

    def _handle_dcmd(self, raw: bytes):
        try:
            payload = spb_pb2.Payload()
            payload.ParseFromString(raw)
        except Exception as exc:
            print(f"[SPB] Failed to parse DCMD: {exc}")
            return

        for metric in payload.metrics:
            name = metric.name
            if name in self._cmd_bool_names:
                if not metric.boolean_value:
                    continue  # only act on True writes; ignore the echo-back False
                code = self._cmd_bool_names[name]
                print(
                    f"[SPB] DCMD Cmd/CntrlCmd/{code}=True | "
                    f"state={self._state}"
                )
                # Echo True — confirms command received.
                self._publish_ddata({name: (MetricDataType.Boolean, True)})
                self._execute_cntrl_cmd(code)
                # Echo False for all cmd tags — one-hot reset so SCADA tag browser is clean.
                self._publish_ddata({
                    self._m(suffix): (MetricDataType.Boolean, False)
                    for suffix in _CMD_BOOL_TAGS.values()
                })
            else:
                print(f"[SPB] DCMD ignored (unknown metric): {name}")

    def _execute_cntrl_cmd(self, code: str):
        if code == CMD_RESET:
            if self._state == STATE_COMPLETE:
                self._set_state(STATE_IDLE)
            else:
                print(f"[SPB] Reset ignored: state must be Complete (got {self._state})")
            return

        if code == CMD_START:
            if self._state != STATE_IDLE:
                print(f"[SPB] Start ignored: state must be Idle (got {self._state})")
                return
            if self._active_alarm_count() > 0:
                print("[SPB] Start ignored: active alarms must be cleared first")
                return
            self._start_detection()
            return

        if code == CMD_STOP:
            self._in_flight = False
            self._set_state(STATE_IDLE)
            return

        if code == CMD_CLEAR:
            if self._state != STATE_ABORTED:
                print(f"[SPB] Clear ignored: state must be Aborted (got {self._state})")
                return
            for c in list(self._alarm_states.keys()):
                if self._alarm_states[c] != ALARM_NORMAL:
                    self._clear_alarm(c)
            self._set_state(STATE_IDLE)
            return

        print(f"[SPB] Unsupported Cmd/CntrlCmd code {code}")

    # ------------------------------------------------------------------
    # PackML state machine
    # ------------------------------------------------------------------

    def _set_state(self, new_state: str):
        if new_state == self._state:
            return
        print(f"[SPB] State: {self._state} → {new_state}")
        self._state = new_state
        self._publish_ddata({
            self._m(suffix): (MetricDataType.Boolean, new_state == state_code)
            for state_code, suffix in _STATE_BOOL_TAGS.items()
        })

    # ------------------------------------------------------------------
    # Detection cycle
    # ------------------------------------------------------------------

    def _start_detection(self):
        self._set_state(STATE_EXECUTE)
        self._in_flight = True
        threading.Thread(target=self._run_detection, daemon=True).start()
        threading.Timer(self._detection_timeout, self._timeout_check).start()

    def _run_detection(self):
        try:
            if self._frame_provider is not None:
                # GUI mode: share the live VideoWorker pipeline.
                pass_, pellet_px, foreign_px, pellet_count = self._frame_provider()
            else:
                # Headless mode: open DepthAI directly for this cycle.
                r = CameraDetector(self._cam_cfg).capture_one_frame()
                pass_, pellet_px, foreign_px, pellet_count = (
                    r.pass_, r.pellet_px, r.foreign_px, r.pellet_count
                )
        except (RuntimeError, queue.Empty) as exc:
            print(f"[SPB] CameraOffline: {exc}")
            if self._in_flight:
                self._in_flight = False
                self._set_aborted(8001)
            return

        if not self._in_flight:
            return  # timed out or stopped before we finished

        self._in_flight = False
        ts_ms = int(time.time() * 1000)
        result_str = "PASS" if pass_ else "REJECT"
        print(
            f"[SPB] Detection complete: {result_str} | "
            f"pellet_px={pellet_px} foreign_px={foreign_px}"
        )
        self._publish_ddata({
            self._m("Result/Last/Pass"):              (MetricDataType.Boolean, pass_),
            self._m("Result/Last/PelletCount"):       (MetricDataType.Int32,   pellet_count),
            self._m("Result/Last/PelletPixelCount"):  (MetricDataType.Int32,   pellet_px),
            self._m("Result/Last/ForeignPixelCount"): (MetricDataType.Int32,   foreign_px),
            self._m("Result/Last/TimestampMs"):       (MetricDataType.Int64,   ts_ms),
        })
        self._set_state(STATE_COMPLETE)

    def push_result(self, pass_: bool, pellet_px: int, foreign_px: int, pellet_count: int):
        """Publish a live detection result immediately, outside any SCADA cycle.

        Called by camera_gui._update_detection() on every frame when the bridge
        is running in shared-pipeline mode, so SCADA sees a continuous stream of
        Result/Last/* tags without needing to trigger individual Start commands.
        """
        if not self._connected:
            return
        ts_ms = int(time.time() * 1000)
        self._publish_ddata({
            self._m("Result/Last/Pass"):              (MetricDataType.Boolean, pass_),
            self._m("Result/Last/PelletCount"):       (MetricDataType.Int32,   pellet_count),
            self._m("Result/Last/PelletPixelCount"):  (MetricDataType.Int32,   pellet_px),
            self._m("Result/Last/ForeignPixelCount"): (MetricDataType.Int32,   foreign_px),
            self._m("Result/Last/TimestampMs"):       (MetricDataType.Int64,   ts_ms),
        })

    def _timeout_check(self):
        if self._in_flight:
            self._in_flight = False
            print("[SPB] DetectionTimeout")
            self._set_aborted(8002)

    # ------------------------------------------------------------------
    # Alarm management
    # ------------------------------------------------------------------

    def _raise_alarm(self, code: int):
        if code not in ALARM_DEFINITIONS or self._alarm_states[code] == ALARM_UNACK:
            return
        priority, message = ALARM_DEFINITIONS[code]
        now_ms = int(time.time() * 1000)
        self._alarm_states[code] = ALARM_UNACK
        self._alarm_onsets[code] = now_ms
        print(f"[SPB] ALARM {code} ({message}) priority={priority} raised at {now_ms}")
        self._publish_ddata({
            self._m(f"Alarm/Active/{code}/State"):   (MetricDataType.Int32, ALARM_UNACK),
            self._m(f"Alarm/Active/{code}/OnsetMs"): (MetricDataType.Int64, now_ms),
            self._m("Alarm/Summary/ActiveCount"):    (MetricDataType.Int32, self._active_alarm_count()),
        })

    def _clear_alarm(self, code: int):
        if code not in ALARM_DEFINITIONS or self._alarm_states[code] == ALARM_NORMAL:
            return
        self._alarm_states[code] = ALARM_NORMAL
        _, message = ALARM_DEFINITIONS[code]
        print(f"[SPB] ALARM {code} ({message}) cleared")
        self._publish_ddata({
            self._m(f"Alarm/Active/{code}/State"): (MetricDataType.Int32, ALARM_NORMAL),
            self._m("Alarm/Summary/ActiveCount"):  (MetricDataType.Int32, self._active_alarm_count()),
        })

    def _active_alarm_count(self) -> int:
        return sum(1 for s in self._alarm_states.values() if s != ALARM_NORMAL)

    def _set_aborted(self, alarm_code: int):
        self._raise_alarm(alarm_code)
        self._set_state(STATE_ABORTED)

    # ------------------------------------------------------------------
    # Primary Host monitoring → Safe State
    # ------------------------------------------------------------------

    def _handle_primary_state(self, raw: bytes):
        text = raw.decode("utf-8", errors="replace").strip()
        online: Optional[bool] = None
        try:
            obj = json.loads(text)
            if isinstance(obj, dict) and "online" in obj:
                online = bool(obj["online"])
        except Exception:
            pass
        if online is None:
            online = text.upper() == "ONLINE"

        if self._primary_seen and online == self._primary_online:
            return
        self._primary_seen = True

        if online:
            self._primary_online = True
            print(f"[SPB] Primary host '{self._primary_host_id}' ONLINE")
            self._clear_alarm(8003)
        else:
            self._primary_online = False
            print(f"[SPB] Primary host '{self._primary_host_id}' OFFLINE — entering Safe State")
            self._set_aborted(8003)

    # ------------------------------------------------------------------
    # Heartbeat loop — blocking main loop
    # ------------------------------------------------------------------

    def run(self):
        """Block until stop() is called, toggling heartbeat on each interval."""
        print("[SPB] Running — press Ctrl+C to stop")
        try:
            while not self._stop_event.is_set():
                self._heartbeat = not self._heartbeat
                self._publish_ddata({
                    self._m("Status/Heartbeat"): (MetricDataType.Boolean, self._heartbeat),
                })
                self._stop_event.wait(self._heartbeat_interval)
        finally:
            self._shutdown()

    def stop(self):
        self._stop_event.set()

    def _shutdown(self):
        try:
            self._mqtt.disconnect()
        except Exception:
            pass
        try:
            self._mqtt.loop_stop()
        except Exception:
            pass
        print("[SPB] Camera bridge stopped")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    bridge = CameraSpbBridge()
    try:
        bridge.run()
    except KeyboardInterrupt:
        pass
    finally:
        bridge.stop()


if __name__ == "__main__":
    main()
