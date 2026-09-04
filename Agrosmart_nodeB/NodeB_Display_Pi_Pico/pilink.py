"""
pilink.py — Pi Pico <-> Node B (ESP32) link over UART0 (Serial1)

Wiring:
    ESP32 GPIO17 (TX2) -> Pico GP1 (UART0 RX)
    ESP32 GPIO16 (RX2) <- Pico GP0 (UART0 TX)

Frame from Node B:
    [0x7E][TYPE][LEN_LO][LEN_HI][...DATA...][CRC8][0x7F]
    TYPE 0x01 = PKT_HEALTH   -> raw SystemHealthState (104 bytes)
                                 dispatched every HEALTH_CHECK_INTERVAL_MS
    TYPE 0x02 = PKT_SENSOR   -> raw MasterSensorRecord (91 bytes, packed)
                                 dispatched every decision_task cycle
    TYPE 0x03 = PKT_DECISION -> raw IrrigationDecision (92 bytes)
                                 dispatched every decision_task cycle
    TYPE 0x04 = PKT_CONFIG   -> raw FarmProfile (52 bytes)
                                 dispatched once at boot + on STATUS_REQUEST

Commands sent Pico -> Node B (single byte, no framing):
    0xA1 = IRRIGATE_NOW
    0xA2 = EMERGENCY_STOP
    0xA3 = STATUS_REQUEST
"""

import ustruct as struct
from machine import UART, Pin

PKT_START = 0x7E
PKT_END = 0x7F
PKT_HEALTH = 0x01
PKT_SENSOR = 0x02
PKT_DECISION = 0x03
PKT_CONFIG = 0x04

CMD_IRRIGATE_NOW = 0xA1
CMD_EMERGENCY_STOP = 0xA2
CMD_STATUS_REQUEST = 0xA3

HTASK_COUNT = 7
HEALTH_QUEUE_COUNT = 9

# Exact byte layout of SystemHealthState (ESP32 GCC natural alignment).
# '<' = little-endian, no auto-padding — pad bytes ('x') inserted by hand
# to match the C compiler's alignment of uint32_t/float members to 4 bytes.
_HEALTH_FMT = (
    "<"
    "II"      # uptime_s, reboot_count                    (0-7)
    "B"       # reboot_reason_last                        (8)
    "8s"      # firmware_version[8]                        (9-16)
    "3x"      # pad to 20
    "II"      # heap_free_bytes, heap_free_min_ever        (20-27)
    "B"       # heap_severity                              (28)
    "B"       # bus_status_mask                            (29)
    "BBBBBB"  # i2c0_ok,i2c1_ok,hspi_sd_ok,vspi_lora_ok,rain_gpio_ok,wifi_ok (30-35)
    "b"       # wifi_rssi                                  (36)
    "3x"      # pad to 40
    "II"      # node_a_last_seen_ms, node_c_last_seen_ms   (40-47)
    "BB"      # node_a_alive, node_c_alive                 (48-49)
    "bb"      # node_a_rssi_last, node_c_rssi_last         (50-51)
    "f"       # nodec_battery_v                            (52-55)
    "BBB"     # nodec_battery_ok, nodec_seen, nodec_status_flag (56-58)
    "B"       # task_alive_mask                            (59)
    "7H"      # task_stack_min[HTASK_COUNT]                (60-73)
    "9B"      # queue_fill_pct[HEALTH_QUEUE_COUNT]         (74-82)
    "1x"      # pad to 84
    "II"      # sd_total_mb, sd_used_mb                    (84-91)
    "B"       # sd_used_pct                                (92)
    "B"       # sd_storage_critical                        (93)
    "B"       # global_health_flag                         (94)
    "B"       # consecutive_critical_heap                  (95)
    "7B"      # task_stale_count[HTASK_COUNT]              (96-102)
    "B"       # pending_reboot_reason                      (103)
)
HEALTH_STRUCT_SIZE = struct.calcsize(_HEALTH_FMT)  # must be 104

_HEALTH_FIELDS = (
    "uptime_s", "reboot_count", "reboot_reason_last", "firmware_version",
    "heap_free_bytes", "heap_free_min_ever", "heap_severity",
    "bus_status_mask",
    "i2c0_ok", "i2c1_ok", "hspi_sd_ok", "vspi_lora_ok", "rain_gpio_ok", "wifi_ok",
    "wifi_rssi",
    "node_a_last_seen_ms", "node_c_last_seen_ms",
    "node_a_alive", "node_c_alive",
    "node_a_rssi_last", "node_c_rssi_last",
    "nodec_battery_v",
    "nodec_battery_ok", "nodec_seen", "nodec_status_flag",
    "task_alive_mask",
) + tuple("task_stack_min_%d" % i for i in range(HTASK_COUNT)) + tuple(
    "queue_fill_pct_%d" % i for i in range(HEALTH_QUEUE_COUNT)
) + (
    "sd_total_mb", "sd_used_mb", "sd_used_pct", "sd_storage_critical",
    "global_health_flag", "consecutive_critical_heap",
) + tuple("task_stale_count_%d" % i for i in range(HTASK_COUNT)) + (
    "pending_reboot_reason",
)

# ── MasterSensorRecord (structs.h) — __attribute__((packed)), no padding ──
_SENSOR_FMT = "<24sIIIB7fB6fB"
SENSOR_STRUCT_SIZE = struct.calcsize(_SENSOR_FMT)  # must be 91

_SENSOR_FIELDS = (
    "record_id", "timestamp_epoch", "boot_time_ms", "record_sequence",
    "timestamp_valid",
    "vwc_percent", "soil_temp", "ec", "ph", "nitrogen", "phosphorus", "potassium",
    "gps_valid",
    "latitude", "longitude",
    "air_temp", "humidity", "pressure", "rainfall",
    "health_flag",
)

# ── IrrigationDecision (structs.h) — natural GCC alignment ──
_DECISION_FMT = "<24s24sIIIB3xffffffBBB1x"
DECISION_STRUCT_SIZE = struct.calcsize(_DECISION_FMT)  # must be 92

_DECISION_FIELDS = (
    "decision_id", "record_id",
    "timestamp_epoch", "boot_time_ms", "record_sequence", "timestamp_valid",
    "taw_mm", "raw_mm", "deficit_mm", "net_required_mm", "gross_required_l", "vpd_kpa",
    "irrigation_required", "status_flag", "decision_reason",
)

# ── FarmProfile (structs.h) — natural GCC alignment ──
_CONFIG_FMT = "<BBHH2xfBB2x8fHB1x"
CONFIG_STRUCT_SIZE = struct.calcsize(_CONFIG_FMT)  # must be 52

_CONFIG_FIELDS = (
    "profile_version", "reserved", "farm_id", "field_id", "field_area_m2",
    "crop_id", "growth_stage",
    "root_depth_m", "field_capacity_vwc", "wilting_point_vwc", "mad_percent",
    "irrigation_efficiency", "max_daily_l", "max_dose_l", "rain_lock_mm",
    "stab_delay_min", "default_zone_id",
)


def crc8(data):
    c = 0
    for b in data:
        c ^= b
    return c & 0xFF


class PiLink:
    def __init__(self, uart_id=0, tx=Pin(0), rx=Pin(1), baud=115200):
        self.uart = UART(uart_id, baudrate=baud, tx=tx, rx=rx)
        self._buf = bytearray()
        self._state = "WAIT_START"
        self._ptype = 0
        self._plen = 0
        self._data = b""
        self.last_health = None     # dict, most recent parsed SystemHealthState
        self.last_sensor = None     # dict, most recent parsed MasterSensorRecord
        self.last_decision = None   # dict, most recent parsed IrrigationDecision
        self.last_config = None     # dict, most recent parsed FarmProfile
        self.packets_ok = 0
        self.packets_bad = 0

    def send_command(self, cmd):
        self.uart.write(bytes([cmd]))

    def irrigate_now(self):
        self.send_command(CMD_IRRIGATE_NOW)

    def emergency_stop(self):
        self.send_command(CMD_EMERGENCY_STOP)

    def status_request(self):
        self.send_command(CMD_STATUS_REQUEST)

    def _parse_health(self, data):
        if len(data) != HEALTH_STRUCT_SIZE:
            return None
        vals = struct.unpack(_HEALTH_FMT, data)
        h = dict(zip(_HEALTH_FIELDS, vals))
        h["firmware_version"] = h["firmware_version"].split(b"\x00")[0].decode()
        h["task_stack_min"] = [h.pop("task_stack_min_%d" % i) for i in range(HTASK_COUNT)]
        h["queue_fill_pct"] = [h.pop("queue_fill_pct_%d" % i) for i in range(HEALTH_QUEUE_COUNT)]
        h["task_stale_count"] = [h.pop("task_stale_count_%d" % i) for i in range(HTASK_COUNT)]
        return h

    def _parse_sensor(self, data):
        if len(data) != SENSOR_STRUCT_SIZE:
            return None
        vals = struct.unpack(_SENSOR_FMT, data)
        s = dict(zip(_SENSOR_FIELDS, vals))
        s["record_id"] = s["record_id"].split(b"\x00")[0].decode()
        return s

    def _parse_decision(self, data):
        if len(data) != DECISION_STRUCT_SIZE:
            return None
        vals = struct.unpack(_DECISION_FMT, data)
        d = dict(zip(_DECISION_FIELDS, vals))
        d["decision_id"] = d["decision_id"].split(b"\x00")[0].decode()
        d["record_id"] = d["record_id"].split(b"\x00")[0].decode()
        return d

    def _parse_config(self, data):
        if len(data) != CONFIG_STRUCT_SIZE:
            return None
        vals = struct.unpack(_CONFIG_FMT, data)
        return dict(zip(_CONFIG_FIELDS, vals))

    def _handle_frame(self, ptype, data):
        if ptype == PKT_HEALTH:
            h = self._parse_health(data)
            if h is not None:
                self.last_health = h
                return "health"
        elif ptype == PKT_SENSOR:
            s = self._parse_sensor(data)
            if s is not None:
                self.last_sensor = s
                return "sensor"
        elif ptype == PKT_DECISION:
            d = self._parse_decision(data)
            if d is not None:
                self.last_decision = d
                return "decision"
        elif ptype == PKT_CONFIG:
            c = self._parse_config(data)
            if c is not None:
                self.last_config = c
                return "config"
        return None

    def poll(self):
        """Call every loop tick. Returns a list of event names ('health',
        'sensor', 'decision', 'config') for every complete frame parsed
        during this call — self.last_* is updated before the event for
        that type appears in the list."""
        n = self.uart.any()
        if not n:
            return []
        chunk = self.uart.read(n)
        if not chunk:
            return []

        events = []
        for byte in chunk:
            if self._state == "WAIT_START":
                if byte == PKT_START:
                    self._state = "WAIT_TYPE"
            elif self._state == "WAIT_TYPE":
                self._ptype = byte
                self._state = "WAIT_LEN_LO"
            elif self._state == "WAIT_LEN_LO":
                self._plen = byte
                self._state = "WAIT_LEN_HI"
            elif self._state == "WAIT_LEN_HI":
                self._plen |= (byte << 8)
                self._buf = bytearray()
                self._state = "DATA" if self._plen > 0 else "CRC"
            elif self._state == "DATA":
                self._buf.append(byte)
                if len(self._buf) >= self._plen:
                    self._state = "CRC"
            elif self._state == "CRC":
                expected = crc8(self._buf)
                if byte == expected:
                    ev = self._handle_frame(self._ptype, bytes(self._buf))
                    if ev:
                        events.append(ev)
                    self.packets_ok += 1
                else:
                    self.packets_bad += 1
                self._state = "WAIT_END"
            elif self._state == "WAIT_END":
                # byte should be PKT_END; resync regardless
                self._state = "WAIT_START"
        return events
