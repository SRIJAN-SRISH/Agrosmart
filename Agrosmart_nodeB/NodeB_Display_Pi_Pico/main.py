from machine import Pin, SPI
import time
from ili9341 import ILI9341, color565
from pilink import PiLink

# --- Node B UART link (UART0: GP0 TX, GP1 RX) ---
link = PiLink(uart_id=0, tx=Pin(0), rx=Pin(1), baud=115200)

# --- TFT setup (SPI0) ---
tft_spi = SPI(0, baudrate=40000000, sck=Pin(18), mosi=Pin(19), miso=Pin(16))
tft = ILI9341(tft_spi, Pin(17), Pin(20), Pin(21), width=240, height=320)

# --- Touch setup (SPI1) - polling, no IRQ ---
touch_spi = SPI(1, baudrate=1000000, sck=Pin(10), mosi=Pin(11), miso=Pin(12))
t_cs = Pin(13, Pin.OUT, value=1)
CMD_X, CMD_Y, CMD_Z1 = 0xD0, 0x90, 0xB0
TX_MIN, TX_MAX = 300, 3800
TY_MIN, TY_MAX = 300, 3800
Z_THRESH = 300  # pressure threshold - raise/lower if too sensitive/insensitive


def read_raw(cmd):
    t_cs.value(0)
    touch_spi.write(bytearray([cmd]))
    buf = bytearray(2)
    touch_spi.readinto(buf)
    t_cs.value(1)
    return ((buf[0] << 8) | buf[1]) >> 3


def get_touch():
    z1 = read_raw(CMD_Z1)
    if z1 < Z_THRESH:
        return None
    x = read_raw(CMD_X)
    y = read_raw(CMD_Y)
    sx = int((x - TX_MIN) * 240 / (TX_MAX - TX_MIN))
    sy = int((y - TY_MIN) * 320 / (TY_MAX - TY_MIN))
    sx = 240 - max(0, min(239, sx))  # flip as needed for your rotation
    sy = 320 - max(0, min(319, sy))
    return (sx, sy)


# --- Colors ---
C_BG      = color565(13, 27, 42)     # deep navy background
C_SURFACE = color565(20, 34, 51)     # splash / panel surface
C_CARD    = color565(28, 49, 72)     # card background
C_BORDER  = color565(37, 62, 88)     # dividers / hairlines
C_ACCENT  = color565(0, 212, 154)    # teal-green accent (OK)
C_WARN    = color565(245, 166, 35)   # amber
C_FAULT   = color565(255, 82, 82)    # red
C_TEXT    = color565(216, 234, 247)  # soft near-white for values
C_MUTED   = color565(94, 130, 160)   # grey-blue for labels
C_NAV     = color565(4, 11, 19)      # nav / status bar background
C_NAV_ACT = color565(7, 20, 34)      # active tab highlight
C_OK      = C_ACCENT

TINT_ACCENT = color565(0, 50, 38)
TINT_WARN   = color565(55, 38, 0)
TINT_FAULT  = color565(55, 0, 0)
BTN_ACCENT  = color565(0, 45, 32)
BTN_FAULT   = color565(50, 0, 0)
DEC_IRR     = color565(40, 25, 0)    # decision card bg (irrigate)
DEC_SKIP    = color565(0, 30, 20)    # decision card bg (skip)

SW, SH = 240, 320
SBAR, NBAR = 22, 38
CY = SBAR
NP = 5
tabs = ["LIVE", "IRRIG", "ENVIR", "SYS", "CFG"]

# ============================================================
# Live data model — fed entirely by frames from Node B over UART2:
#   PKT_HEALTH   (every HEALTH_CHECK_INTERVAL_MS) -> apply_health()
#   PKT_SENSOR   (every decision_task cycle)       -> apply_sensor()
#   PKT_DECISION (every decision_task cycle)       -> apply_decision()
#   PKT_CONFIG   (once at boot + on request)       -> apply_config()
# Nothing in D is fabricated — every field is either straight off the
# wire or trivially derived (e.g. valve state from status_flag).
# ============================================================
REASON_RAIN_LOCK = 0x10
IRRIGATION_IN_PROGRESS = 0x02

DECISION_REASON_TEXT = {
    0x00: "NONE", 0x01: "DEFICIT", 0x10: "RAIN LOCK", 0x11: "LOW BATT",
    0x12: "SENSOR FAULT", 0x13: "LORA FAULT", 0x14: "SD FAULT",
    0x15: "MANUAL", 0x16: "CONFIG FAULT", 0x17: "DAILY LIMIT",
    0x18: "STAB DELAY", 0x19: "BELOW THRESH", 0x1A: "INTERNAL FAULT",
}

D = {
    # -- from PKT_SENSOR (MasterSensorRecord) --
    "vwc": 0.0, "soilT": 0.0, "ec": 0.0, "ph": 0.0,
    "n": 0, "p": 0, "k": 0,
    "airT": 0.0, "hum": 0.0, "pres": 0.0, "rain": 0.0,

    # -- from PKT_DECISION (IrrigationDecision) --
    "irrigate": False, "deficit": 0.0, "dose": 0.0, "vpd": 0.0,
    "valve": False, "lastDose": 0.0, "reasonTxt": "--",
    "rainActive": False,

    # -- from PKT_CONFIG (FarmProfile, static) --
    "fc": 0, "wp": 0, "rainLock": 0.0,
    "farmId": 0, "fieldId": 0, "area": 0,
    "mad": 0, "maxDose": 0, "efficiency": 0, "zone": 0,
    "cropId": 0, "stageId": 0,

    # -- from PKT_HEALTH (SystemHealthState) --
    "wifiOk": False, "sdOk": False, "rtcOk": False, "nodeCok": False,
    "heap": 0, "sdPct": 0, "rssiA": 0, "rssiW": 0, "rssiC": 0,
    "health": 0xFF, "uptime_s": 0, "reboot_count": 0,
}

page = 0
health_seen = False   # becomes True on first valid PKT_HEALTH frame
sensor_seen = False   # becomes True on first valid PKT_SENSOR frame
config_seen = False   # becomes True on first valid PKT_CONFIG frame


def apply_health(h):
    """Copy fields from a parsed SystemHealthState dict into D."""
    D["wifiOk"] = bool(h["wifi_ok"])
    D["sdOk"] = bool(h["hspi_sd_ok"])
    D["rtcOk"] = bool(h["i2c0_ok"])  # RTC shares I2C-0 with BMP280
    D["nodeCok"] = bool(h["node_c_alive"])
    D["heap"] = h["heap_free_bytes"]
    D["sdPct"] = h["sd_used_pct"]
    D["rssiA"] = h["node_a_rssi_last"]
    D["rssiW"] = h["wifi_rssi"]
    D["rssiC"] = h["node_c_rssi_last"]
    D["health"] = h["global_health_flag"]
    D["uptime_s"] = h["uptime_s"]
    D["reboot_count"] = h["reboot_count"]


def apply_sensor(s):
    """Copy fields from a parsed MasterSensorRecord dict into D."""
    D["vwc"] = s["vwc_percent"]
    D["soilT"] = s["soil_temp"]
    D["ec"] = s["ec"]
    D["ph"] = s["ph"]
    D["n"] = s["nitrogen"]
    D["p"] = s["phosphorus"]
    D["k"] = s["potassium"]
    D["airT"] = s["air_temp"]
    D["hum"] = s["humidity"]
    D["pres"] = s["pressure"]
    D["rain"] = s["rainfall"]


def apply_decision(d):
    """Copy fields from a parsed IrrigationDecision dict into D."""
    D["irrigate"] = bool(d["irrigation_required"])
    D["deficit"] = d["deficit_mm"]
    D["dose"] = d["gross_required_l"]
    D["vpd"] = d["vpd_kpa"]
    D["valve"] = (d["status_flag"] == IRRIGATION_IN_PROGRESS)
    D["rainActive"] = (d["decision_reason"] == REASON_RAIN_LOCK)
    D["reasonTxt"] = DECISION_REASON_TEXT.get(d["decision_reason"], "?")
    if d["irrigation_required"]:
        D["lastDose"] = d["gross_required_l"]


def apply_config(c):
    """Copy fields from a parsed FarmProfile dict into D. Static — sent once."""
    D["fc"] = c["field_capacity_vwc"]
    D["wp"] = c["wilting_point_vwc"]
    D["rainLock"] = c["rain_lock_mm"]
    D["farmId"] = c["farm_id"]
    D["fieldId"] = c["field_id"]
    D["area"] = c["field_area_m2"]
    D["mad"] = c["mad_percent"]
    D["maxDose"] = c["max_dose_l"]
    D["efficiency"] = c["irrigation_efficiency"]
    D["zone"] = c["default_zone_id"]
    D["cropId"] = c["crop_id"]
    D["stageId"] = c["growth_stage"]


# ============================================================
# Drawing helpers
# ============================================================

def _round(x, y, w, h):
    tft.pixel(x, y, C_BG)
    tft.pixel(x + w - 1, y, C_BG)
    tft.pixel(x, y + h - 1, C_BG)
    tft.pixel(x + w - 1, y + h - 1, C_BG)


def card(x, y, w, h, color=C_CARD, border=None):
    tft.fill_rect(x, y, w, h, color)
    if border:
        tft.rect(x, y, w, h, border)
    _round(x, y, w, h)


def text_b(s, x, y, c):
    tft.text(s, x, y, c)
    tft.text(s, x + 1, y, c)


def row(x, y, w, lbl, val, vc=C_TEXT):
    tft.text(lbl, x, y, C_MUTED)
    vx = x + w - len(val) * 8
    tft.text(val, vx, y, vc)


def divider(y):
    tft.hline(8, y, SW - 16, C_BORDER)


def _pill_bg(color):
    if color == C_ACCENT:
        return TINT_ACCENT
    if color == C_WARN:
        return TINT_WARN
    if color == C_FAULT:
        return TINT_FAULT
    return C_CARD


def pill(x, y, text, color):
    bg = _pill_bg(color)
    w = len(text) * 8 + 10
    tft.fill_rect(x, y, w, 14, bg)
    tft.rect(x, y, w, 14, color)
    tft.text(text, x + 5, y + 3, color)
    return w


def _btn_bg(color):
    if color == C_FAULT:
        return BTN_FAULT
    if color == C_ACCENT:
        return BTN_ACCENT
    return C_CARD


def btn(x, y, w, h, text, color):
    tft.fill_rect(x, y, w, h, _btn_bg(color))
    tft.rect(x, y, w, h, color)
    tw = len(text) * 8
    tft.text(text, x + (w - tw) // 2, y + (h - 8) // 2, color)
    _round(x, y, w, h)


def hbar(x, y, w, pct, color):
    pct = max(0.0, min(1.0, pct))
    tft.fill_rect(x, y, w, 6, C_CARD)
    fw = int(w * pct)
    if fw > 0:
        tft.fill_rect(x, y, fw, 6, color)


def icon_wifi(cx, cy, color, ok=True):
    col = color if ok else C_FAULT
    tft.ellipse(cx, cy + 2, 6, 5, col, False)
    tft.ellipse(cx, cy + 2, 4, 3, col, False)
    tft.fill_rect(cx - 1, cy + 3, 2, 2, col)


def icon_lora(cx, cy, color, ok=True):
    col = color if ok else C_FAULT
    tft.fill_rect(cx - 6, cy + 3, 2, 3, col)
    tft.fill_rect(cx - 2, cy + 1, 2, 5, col)
    tft.fill_rect(cx + 2, cy - 1, 2, 7, col)


def icon_sd(cx, cy, color, ok=True):
    col = color if ok else C_FAULT
    tft.rect(cx - 5, cy - 5, 10, 12, col)
    tft.fill_rect(cx - 5, cy - 5, 4, 3, col)


def icon_clock(cx, cy, color, ok=True):
    col = color if ok else C_FAULT
    tft.ellipse(cx, cy, 6, 6, col, False)
    tft.line(cx, cy, cx, cy - 4, col)
    tft.line(cx, cy, cx + 3, cy, col)


def nav_icon(tx, tw, i, active):
    cx = tx + tw // 2
    cy = SH - NBAR + 8
    col = C_ACCENT if active else C_MUTED
    if i == 0:
        tft.line(cx, cy - 4, cx + 4, cy, col)
        tft.line(cx + 4, cy, cx, cy + 4, col)
        tft.line(cx, cy + 4, cx - 4, cy, col)
        tft.line(cx - 4, cy, cx, cy - 4, col)
    elif i == 1:
        tft.ellipse(cx, cy, 4, 4, col, True)
    elif i == 2:
        tft.ellipse(cx - 2, cy, 3, 3, col, False)
        tft.ellipse(cx + 2, cy, 3, 3, col, False)
    elif i == 3:
        tft.ellipse(cx, cy, 4, 4, col, False)
        tft.pixel(cx, cy, col)
    elif i == 4:
        tft.hline(cx - 4, cy - 3, 8, col)
        tft.hline(cx - 4, cy, 8, col)
        tft.hline(cx - 4, cy + 3, 8, col)


def fmt_uptime(s):
    h = s // 3600
    m = (s % 3600) // 60
    sec = s % 60
    return "%02d:%02d:%02d" % (h, m, sec)


def status_bar():
    tft.fill_rect(0, 0, SW, SBAR, C_NAV)
    tft.hline(0, SBAR - 1, SW, C_BORDER)
    tft.text("NODE B", 6, 8, C_ACCENT)
    t = fmt_uptime(D["uptime_s"])[:5]
    tft.text(t, SW - len(t) * 8 - 22, 8, C_MUTED)
    ix = SW - len(t) * 8 - 30
    icon_clock(ix, SBAR // 2, C_ACCENT, D["rtcOk"]); ix -= 22
    sd_col = C_WARN if D["sdPct"] > 85 else C_ACCENT
    icon_sd(ix, SBAR // 2, sd_col, D["sdOk"]); ix -= 22
    icon_lora(ix, SBAR // 2, C_ACCENT, D["nodeCok"]); ix -= 22
    icon_wifi(ix, SBAR // 2, C_ACCENT, D["wifiOk"])
    if not health_seen:
        pill(SW // 2 - 26, SBAR + 2, "NO LINK", C_FAULT)


def nav_bar(active):
    ny = SH - NBAR
    tft.fill_rect(0, ny, SW, NBAR, C_NAV)
    tft.hline(0, ny, SW, C_BORDER)
    tw = SW // NP
    for i in range(NP):
        tx = i * tw
        if i == active:
            tft.fill_rect(tx, ny, tw, NBAR, C_NAV_ACT)
            tft.fill_rect(tx, ny, 2, NBAR, C_ACCENT)
        col = C_ACCENT if i == active else C_MUTED
        nav_icon(tx, tw, i, i == active)
        lbl = tabs[i]
        lx = tx + (tw - len(lbl) * 8) // 2
        tft.text(lbl, lx, ny + 22, col)
        if i < NP - 1:
            tft.vline(tx + tw - 1, ny + 6, NBAR - 12, C_BORDER)


# ============================================================
# Pages
# ============================================================

def page_live():
    x = 8
    y = CY + 6
    cw = (SW - 20) // 3

    tft.text("SOIL MOISTURE", x, y, C_MUTED); y += 12
    vwc = D["vwc"]
    vc = C_FAULT if vwc < 20 else (C_WARN if vwc < 28 else C_ACCENT)
    text_b("%.1f%%" % vwc, x, y, vc); y += 16

    span = D["fc"] - D["wp"]
    pct = (vwc - D["wp"]) / span if span > 0 else 0.0
    hbar(x, y, SW - 16, pct, vc); y += 10
    tft.text("WP %d%%" % D["wp"], x, y, C_MUTED)
    fcs = "FC %d%%" % D["fc"]
    tft.text(fcs, SW - 8 - len(fcs) * 8, y, C_MUTED); y += 16

    labels = ["TEMP", "EC", "pH"]
    vals = ["%.1f" % D["soilT"], "%.1f" % D["ec"], "%.1f" % D["ph"]]
    phc = C_ACCENT if 6 <= D["ph"] <= 7.5 else C_WARN
    cols = [C_TEXT, C_TEXT, phc]
    for i in range(3):
        cx = x + i * (cw + 2)
        card(cx, y, cw, 30)
        tft.text(labels[i], cx + 4, y + 4, C_MUTED)
        tft.text(vals[i], cx + 4, y + 17, cols[i])
    y += 34

    nlabels = ["N", "P", "K"]
    nvals = ["%d" % D["n"], "%d" % D["p"], "%d" % D["k"]]
    for i in range(3):
        cx = x + i * (cw + 2)
        card(cx, y, cw, 24)
        tft.text(nlabels[i], cx + 4, y + 4, C_MUTED)
        tft.text(nvals[i], cx + 4, y + 14, C_TEXT)
    y += 30

    divider(y); y += 8

    irr = D["irrigate"]
    dc = C_WARN if irr else C_ACCENT
    dbg = DEC_IRR if irr else DEC_SKIP
    card(x, y, SW - 16, 46, dbg, dc)
    tft.text("DECISION", x + 6, y + 6, C_MUTED)
    pill(x + 80, y + 4, "IRRIGATE" if irr else D["reasonTxt"], dc)
    tft.text("Deficit %.1f mm" % D["deficit"], x + 6, y + 22, dc)
    tft.text("Dose  %.1f L" % D["dose"], x + 6, y + 34, dc)


irrig_btn_y = 0
stop_btn_y = 0


def page_irrig():
    global irrig_btn_y, stop_btn_y
    x = 8
    y = CY + 6
    w = SW - 16
    hw = (SW - 20) // 2

    row(x, y, w, "VALVE", "OPEN" if D["valve"] else "CLOSED",
        C_FAULT if D["valve"] else C_MUTED); y += 16

    card(x, y, hw, 44)
    tft.text("NODE C", x + 4, y + 4, C_MUTED)
    if D["nodeCok"]:
        pill(x + 4, y + 15, "ONLINE", C_ACCENT)
    else:
        pill(x + 4, y + 15, "OFFLINE", C_FAULT)
    tft.text("%d dBm" % D["rssiC"], x + 4, y + 32, C_MUTED)

    card(x + hw + 4, y, hw, 44)
    tft.text("LAST DOSE", x + hw + 8, y + 4, C_MUTED)
    tft.text("%.1f L" % D["lastDose"], x + hw + 8, y + 17, C_TEXT)
    tft.text(D["reasonTxt"], x + hw + 8, y + 30, C_MUTED)
    y += 50

    irr = D["irrigate"]
    card(x, y, w, 56, DEC_IRR if irr else C_CARD, C_WARN if irr else C_BORDER)
    tft.text("DECISION", x + 6, y + 6, C_MUTED)
    pill(x + w - 90, y + 4, "IRRIGATE" if irr else D["reasonTxt"],
         C_WARN if irr else C_MUTED)
    tft.text("Volume", x + 6, y + 22, C_MUTED)
    vv = "%.1f L" % D["dose"]
    tft.text(vv, x + w - 6 - len(vv) * 8, y + 22, C_WARN if irr else C_TEXT)
    tft.text("Deficit", x + 6, y + 34, C_MUTED)
    dv = "%.1f mm" % D["deficit"]
    tft.text(dv, x + w - 6 - len(dv) * 8, y + 34, C_TEXT)
    tft.text("Zone", x + 6, y + 46, C_MUTED)
    tft.text(str(D["zone"]), x + w - 6 - len(str(D["zone"])) * 8, y + 46, C_TEXT)
    y += 64

    irrig_btn_y = y
    btn(x, y, w, 24, "IRRIGATE NOW", C_ACCENT)
    y += 28
    stop_btn_y = y
    btn(x, y, w, 24, "EMERGENCY STOP", C_FAULT)


def page_environ():
    x = 8
    y = CY + 6
    w = SW - 16
    hw = (SW - 20) // 2

    tft.text("BMP280  CLIMATE", x, y, C_MUTED); y += 14

    card(x, y, w, 38, color565(14, 26, 40), C_ACCENT)
    tft.text("AIR TEMPERATURE", x + 6, y + 5, C_MUTED)
    text_b("%.1f C" % D["airT"], x + 6, y + 19, C_ACCENT)
    y += 44

    card(x, y, hw, 32)
    tft.text("HUMIDITY", x + 4, y + 4, C_MUTED)
    tft.text("%.0f%%" % D["hum"], x + 4, y + 18, C_TEXT)
    card(x + hw + 4, y, hw, 32)
    tft.text("PRESSURE", x + hw + 8, y + 4, C_MUTED)
    tft.text("%.0f hPa" % D["pres"], x + hw + 8, y + 18, C_TEXT)
    y += 38

    vc = C_WARN if D["vpd"] > 2.0 else C_ACCENT
    card(x, y, w, 24, C_CARD, vc)
    tft.text("VPD", x + 6, y + 8, C_MUTED)
    vs = "%.2f kPa" % D["vpd"]
    tft.text(vs, x + w - 6 - len(vs) * 8, y + 8, vc)
    y += 30

    divider(y); y += 8
    tft.text("RAIN GAUGE", x, y, C_MUTED); y += 14

    card(x, y, hw, 32)
    tft.text("TODAY", x + 4, y + 4, C_MUTED)
    tft.text("%.1f mm" % D["rain"], x + 4, y + 18, C_TEXT)
    card(x + hw + 4, y, hw, 32)
    tft.text("LOCK", x + hw + 8, y + 4, C_MUTED)
    if D["rainActive"]:
        pill(x + hw + 8, y + 15, "ACTIVE", C_FAULT)
    else:
        pill(x + hw + 8, y + 15, "INACTIVE", C_MUTED)
    y += 38

    ts = "Thr %.0f  Now %.1f mm" % (D["rainLock"], D["rain"])
    tft.text(ts, x, y, C_MUTED); y += 12
    lock_pct = (D["rain"] / D["rainLock"]) if D["rainLock"] > 0 else 0.0
    hbar(x, y, w, lock_pct, C_ACCENT)


def page_sys():
    x = 8
    y = CY + 6
    w = SW - 16

    tft.text("CONNECTIVITY", x, y, C_MUTED); y += 13
    row(x, y, w, "WiFi", "%d dBm" % D["rssiW"],
        C_ACCENT if D["wifiOk"] else C_FAULT); y += 12
    row(x, y, w, "LoRa A", "%d dBm" % D["rssiA"],
        C_ACCENT if D["rssiA"] > -90 else C_WARN); y += 12
    row(x, y, w, "LoRa C", "%d dBm" % D["rssiC"],
        C_ACCENT if D["nodeCok"] else C_FAULT); y += 12
    row(x, y, w, "Link", "OK" if health_seen else "NO DATA",
        C_ACCENT if health_seen else C_FAULT); y += 15

    divider(y); y += 8
    tft.text("RESOURCES", x, y, C_MUTED); y += 13

    row(x, y, w, "Heap Free", "%d kB" % (D["heap"] // 1024),
        C_ACCENT if D["heap"] > 50000 else C_FAULT); y += 11
    hbar(x, y, w, D["heap"] / 327680, C_ACCENT); y += 12

    row(x, y, w, "SD Used", "%d%%" % D["sdPct"],
        C_WARN if D["sdPct"] > 85 else C_ACCENT); y += 11
    hbar(x, y, w, D["sdPct"] / 100, C_WARN if D["sdPct"] > 85 else C_ACCENT); y += 12

    row(x, y, w, "Uptime", fmt_uptime(D["uptime_s"]), C_MUTED); y += 12
    row(x, y, w, "Reboots", str(D["reboot_count"]), C_MUTED); y += 12
    row(x, y, w, "Health", "0x%02X" % D["health"],
        C_ACCENT if D["health"] == 0 else C_FAULT); y += 15

    divider(y); y += 8
    tft.text("LINK STATS", x, y, C_MUTED); y += 13
    row(x, y, w, "Frames OK", str(link.packets_ok), C_MUTED); y += 12
    row(x, y, w, "Frames Bad", str(link.packets_bad),
        C_MUTED if link.packets_bad == 0 else C_WARN)


CROP_NAMES = {0: "Vegetable", 1: "Grain", 2: "Fruit", 3: "Fiber", 4: "Other"}
STAGE_NAMES = {0: "Initial", 1: "Development", 2: "Mid-Season",
               3: "Late Season", 4: "Harvest"}


def page_config():
    x = 8
    y = CY + 6
    w = SW - 16

    tft.text("FARM PROFILE", x, y, C_MUTED)
    pill(SW - 92, y - 2, "LIVE" if config_seen else "WAITING",
         C_ACCENT if config_seen else C_WARN)
    y += 16
    divider(y); y += 8

    row(x, y, w, "Farm ID", str(D["farmId"])); y += 12
    row(x, y, w, "Field ID", str(D["fieldId"])); y += 12
    row(x, y, w, "Field Area", "%d m2" % D["area"]); y += 12
    row(x, y, w, "Crop", CROP_NAMES.get(D["cropId"], "?")); y += 12
    row(x, y, w, "Stage", STAGE_NAMES.get(D["stageId"], "?")); y += 15

    divider(y); y += 8
    row(x, y, w, "Field Cap", "%d%%" % D["fc"]); y += 12
    row(x, y, w, "Wilt Point", "%d%%" % D["wp"]); y += 12
    row(x, y, w, "MAD", "%d%%" % D["mad"]); y += 12
    row(x, y, w, "Max Dose", "%d L" % D["maxDose"]); y += 12
    row(x, y, w, "Rain Lock", "%d mm" % D["rainLock"]); y += 12
    row(x, y, w, "Efficiency", "%d%%" % D["efficiency"]); y += 16

    btn(x, y, w, 18, "READ-ONLY  EDIT VIA SD", C_MUTED)


page_funcs = [page_live, page_irrig, page_environ, page_sys, page_config]


def redraw():
    tft.fill(C_BG)
    page_funcs[page]()
    status_bar()
    nav_bar(page)
    tft.show()


redraw()
link.status_request()  # ask Node B for an immediate health frame on boot
print("Ready. Waiting for Node B UART link on GP0/GP1 @ 115200...")

last_touch_time = 0
last_redraw_time = time.ticks_ms()
REDRAW_INTERVAL = 1000  # ms — SYS/status-bar values are time-sensitive

# ── Auto-cycle through all 5 pages — temporary fallback while touch is
# being debugged separately. Lets you visually confirm every page renders
# correctly without depending on touch working at all. Touch handling below
# is left fully intact — if touch starts working, it still responds
# immediately; auto-cycle just keeps moving in the background regardless.
AUTO_CYCLE_MS = 4000
last_cycle_time = time.ticks_ms()

while True:
    # -- service Node B UART link every tick --
    for event in link.poll():
        if event == "health":
            if not health_seen:
                health_seen = True
                print("Node B link established.")
            apply_health(link.last_health)
        elif event == "sensor":
            sensor_seen = True
            apply_sensor(link.last_sensor)
        elif event == "decision":
            apply_decision(link.last_decision)
        elif event == "config":
            config_seen = True
            apply_config(link.last_config)

    t = get_touch()
    now = time.ticks_ms()
    if t and time.ticks_diff(now, last_touch_time) > 300:
        sx, sy = t
        print("Touch at (%d,%d)" % (sx, sy))
        if sy >= SH - NBAR:
            tab = sx // (SW // NP)
            if 0 <= tab < NP and tab != page:
                page = tab
                redraw()
            last_touch_time = now
            last_redraw_time = now
        elif page == 1:
            if irrig_btn_y <= sy <= irrig_btn_y + 24:
                print("-> IRRIGATE_NOW sent to Node B")
                link.irrigate_now()
                redraw()
            elif stop_btn_y <= sy <= stop_btn_y + 24:
                print("-> EMERGENCY_STOP sent to Node B")
                link.emergency_stop()
                redraw()
            last_touch_time = now
            last_redraw_time = now

    if time.ticks_diff(now, last_cycle_time) >= AUTO_CYCLE_MS:
        page = (page + 1) % NP
        print("Auto-cycle -> page %d (%s)" % (page, tabs[page]))
        redraw()
        last_cycle_time = now
        last_redraw_time = now
    elif time.ticks_diff(now, last_redraw_time) >= REDRAW_INTERVAL:
        redraw()
        last_redraw_time = now

    time.sleep_ms(20)
