/*
 * web_dashboard.cpp — HTTP + WebSocket dashboard for Tesla FSD ESP32
 *
 * HTTP  :80  → serves the embedded HTML page
 * WS    :81  → pushes JSON state every 1 s; receives control commands
 *
 * All HTML/CSS/JS is embedded as a raw-string literal — no external CDN.
 * State is shared with the CAN task. Reads copy a locked snapshot; writes use
 * the same FreeRTOS critical section as the CAN/button side.
 */

#include "web_dashboard.h"
#include "can_dump.h"
#include "http_can_stream.h"
#include "blackbox.h"
#include "capability.h"
#include "profile_match.h"
#include "prefs.h"
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <WiFi.h>
#include <Arduino.h>
#include <Update.h>
#include <esp_ota_ops.h>
#include <esp_partition.h>

// ── Module state ──────────────────────────────────────────────────────────────
static FSDState  *g_state = nullptr;   // shared with main
static CanDriver **g_can_buses = nullptr; // mode switch, error split, pre-reboot quiesce
static uint8_t g_can_count = 0;
static portMUX_TYPE *g_state_mux = nullptr;

static WebServer        g_http(80);
static WebSocketsServer g_ws(81);

static uint32_t g_start_ms    = 0;
static uint32_t g_last_rx     = 0;
static uint32_t g_last_fps_ms = 0;
static uint32_t g_last_can_seen_ms = 0;
static float    g_fps         = 0.0f;
static float    g_chip_temp_c = 0.0f;
static float    g_chip_temp_max_c = 0.0f;

static void sample_chip_temperature() {
    float t = temperatureRead();
    if (t > -40.0f && t < 150.0f) {
        g_chip_temp_c = t;
        if (g_chip_temp_max_c == 0.0f || t > g_chip_temp_max_c) g_chip_temp_max_c = t;
    }
}

#define CAN_VEHICLE_ALIVE_MS 3000u
#define OTA_ESP32_IMAGE_MAGIC 0xE9u
#define OTA_AUTH_USER "admin"

static void state_enter() {
    if (g_state_mux) portENTER_CRITICAL(g_state_mux);
}

static void state_exit() {
    if (g_state_mux) portEXIT_CRITICAL(g_state_mux);
}

static bool state_copy(FSDState *out) {
    if (g_state == nullptr || out == nullptr) return false;
    state_enter();
    *out = *g_state;
    state_exit();
    return true;
}

static bool ap_has_password(const FSDState *state) {
    return state != nullptr && strlen(state->wifi_pass) >= 8;
}

static bool require_admin_auth(bool challenge_browser = false) {
    FSDState s;
    if (!state_copy(&s) || !ap_has_password(&s)) {
        g_http.send(403, "text/plain", "WiFi AP password required before OTA/restart");
        return false;
    }
    if (g_http.authenticate(OTA_AUTH_USER, s.wifi_pass)) return true;
    if (challenge_browser) {
        g_http.requestAuthentication(BASIC_AUTH, "Tesla-FSD");
    } else {
        g_http.send(401, "text/plain", "Authentication failed");
    }
    return false;
}

// Black-box captures hold recorded CAN (VIN, drive data), so gate the download
// endpoints behind admin auth — but only when an AP password is set. Open setups
// keep the one-click capture-download workflow; password-protected devices don't
// leak persistent captures to other hosts on a shared LAN. Softer than
// require_admin_auth (which hard-blocks when no password is set).
static bool download_auth_ok() {
    FSDState s;
    if (!state_copy(&s) || !ap_has_password(&s)) return true;   // open setup — allow
    if (g_http.authenticate(OTA_AUTH_USER, s.wifi_pass)) return true;
    g_http.requestAuthentication(BASIC_AUTH, "Tesla-FSD");
    return false;
}

// ── Embedded HTML/CSS/JS ──────────────────────────────────────────────────────
// Tesla dark theme; mobile-first (max 480 px); WebSocket on :81
static const char WEB_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<meta name="apple-mobile-web-app-capable" content="yes">
<meta name="apple-mobile-web-app-status-bar-style" content="black-translucent">
<meta name="theme-color" content="#0a0a1a">
<link rel="icon" href="data:,">
<title>BSB Tesla FSD 中文增强版</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
:root{
  --bg:#0a0a1a;--card:#111827;--card2:#1a1f35;
  --accent:#00d4aa;--accent2:#00b894;
  --red:#ff6b6b;--yellow:#ffd93d;--blue:#4dabf7;
  --border:#1e293b;--text:#e2e8f0;--text2:#94a3b8;--text3:#475569
}
body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,sans-serif;
  background:var(--bg);color:var(--text);min-height:100vh}
.wrap{max-width:480px;margin:0 auto;padding:16px 16px 40px}

/* ── Header ── */
.hdr{text-align:center;padding:20px 0 12px;position:relative}
.hdr h1{font-size:1.65em;font-weight:700;
  background:linear-gradient(135deg,var(--accent),var(--blue));
  -webkit-background-clip:text;-webkit-text-fill-color:transparent;
  letter-spacing:-.02em}
.hdr .sub{font-size:.68em;color:var(--text3);margin-top:3px;
  letter-spacing:.1em;text-transform:uppercase}
.cdot{position:absolute;right:0;top:26px;width:10px;height:10px;
  border-radius:50%;background:var(--accent);
  box-shadow:0 0 10px var(--accent);transition:.4s}
.cdot.off{background:var(--red);box-shadow:0 0 10px var(--red)}

/* ── OTA Warning ── */
.ota{display:none;background:rgba(255,107,107,.1);border:1px solid rgba(255,107,107,.4);
  border-radius:12px;padding:12px 16px;margin-bottom:12px;text-align:center;
  color:var(--red);font-weight:700;font-size:.9em;letter-spacing:.04em;
  animation:pulse 1s ease-in-out infinite}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:.55}}

/* ── 14.x firmware warning (static, non-animated) ── */
.warn14x{display:none;background:rgba(252,196,25,.08);border:1px solid rgba(252,196,25,.45);
  border-radius:12px;padding:10px 14px;margin-bottom:12px;color:#fcc419;font-size:.82em;
  line-height:1.4;text-align:left}
.warn14x .w-row{display:flex;justify-content:space-between;align-items:center;gap:12px}
.warn14x .w-msg{flex:1}
.warn14x .w-dismiss{background:transparent;border:1px solid rgba(252,196,25,.5);color:#fcc419;
  padding:5px 10px;border-radius:6px;cursor:pointer;font-size:.8em;font-weight:600;
  white-space:nowrap}
.warn14x .w-dismiss:hover{background:rgba(252,196,25,.15)}

/* ── Error banner ── */
.err{display:none;color:var(--red);text-align:center;font-size:.78em;padding:8px;
  background:rgba(255,107,107,.07);border-radius:10px;margin-bottom:10px;
  border:1px solid rgba(255,107,107,.18)}

/* ── Cards ── */
.card{background:var(--card);border-radius:16px;padding:16px;
  margin-bottom:12px;border:1px solid var(--border)}
.card-head{display:flex;align-items:center;gap:8px;margin-bottom:12px}
.config-section{background:var(--card);border:1px solid var(--border);
  border-radius:16px;margin-bottom:12px;overflow:hidden}
.config-section summary{display:flex;align-items:center;gap:8px;list-style:none;
  padding:16px;cursor:pointer;user-select:none}
.config-section summary::-webkit-details-marker{display:none}
.config-section summary:after{content:"";margin-left:auto;width:9px;height:9px;
  border-right:2px solid var(--text2);border-bottom:2px solid var(--text2);
  transform:rotate(45deg);transition:transform .2s}
.config-section[open] summary:after{transform:rotate(225deg)}
.config-section .config-body{padding:0 12px 12px}
.config-section .card{border-radius:12px;margin-bottom:10px}
.config-section .card:last-child{margin-bottom:0}
.controls-fold{margin-top:10px;border-top:1px solid rgba(255,255,255,.04)}
.controls-fold summary{position:relative;display:block;list-style:none;
  padding:10px 28px 0 0;cursor:pointer;user-select:none;min-height:24px}
.controls-fold summary::-webkit-details-marker{display:none}
.controls-fold summary:after{content:"";position:absolute;right:4px;top:12px;width:9px;height:9px;
  border-right:2px solid var(--text2);border-bottom:2px solid var(--text2);
  transform:rotate(45deg);transition:transform .2s}
.controls-fold[open] summary:after{transform:rotate(225deg);top:16px}
.controls-fold[open] .control-summary{display:none}
.control-summary{color:var(--text2);font-size:.76em;
  display:block;max-width:calc(100% - 8px);white-space:normal;line-height:1.35;padding-right:8px}
.controls-body{padding-top:8px}
.icon{width:28px;height:28px;border-radius:8px;display:flex;
  align-items:center;justify-content:center;font-size:.85em;font-weight:700}
.ic-s{background:rgba(0,212,170,.14);color:var(--accent)}
.ic-b{background:rgba(77,171,247,.14);color:var(--blue)}
.ic-c{background:rgba(255,217,61,.14);color:var(--yellow)}
.ic-d{background:rgba(148,163,184,.14);color:var(--text2)}
.card-head h2{font-size:.78em;font-weight:600;color:var(--text2);
  text-transform:uppercase;letter-spacing:.07em}

/* ── Rows ── */
.row{display:flex;justify-content:space-between;align-items:center;padding:9px 0}
.row+.row{border-top:1px solid rgba(255,255,255,.04)}
.lbl{color:var(--text2);font-size:.85em}
.hint{display:block;color:var(--text2);opacity:.62;font-size:.82em;line-height:1.35;margin-top:2px;max-width:20em}
details select{background:var(--card2);border:1px solid var(--border);color:var(--text);padding:4px 6px;border-radius:4px;font-size:.85em;flex:none}
details input{background:var(--card2);border:1px solid var(--border);color:var(--text);padding:4px;border-radius:4px;text-align:right;width:60px}
details input.cgn{width:38px;margin-left:4px}

/* Speed settings use the full card width, with one limit per row. */
.speed-offset{display:block}
.speed-offset>.lbl{display:block;margin-bottom:10px}
.speed-offset .row{gap:8px}
.speed-offset .row>.lbl{min-width:0}
.speed-bands{margin:6px 0 10px}
.speed-band{display:flex;align-items:center;justify-content:space-between;
  flex-wrap:wrap;gap:6px 12px;padding:8px 0;color:var(--text2);font-size:.82em}
.speed-band+.speed-band{border-top:1px solid rgba(255,255,255,.04)}
.speed-value{display:flex;align-items:center;gap:5px;white-space:nowrap}
.speed-band input{width:64px;min-height:36px;font-size:16px}
.chip-temperature{display:flex;align-items:center;flex-wrap:wrap;gap:8px 14px;padding:12px 14px}
.chip-temperature .temp-reading{display:flex;align-items:baseline;gap:6px;font-size:.82em}
.chip-temperature b{font-variant-numeric:tabular-nums;white-space:nowrap}
.chip-temperature .pill{margin-left:auto;white-space:nowrap}

/* ── Pills ── */
.pill{display:inline-flex;align-items:center;gap:5px;
  padding:3px 10px;border-radius:20px;font-size:.8em;font-weight:600}
.pill.on{background:rgba(0,212,170,.14);color:var(--accent)}
.pill.off{background:rgba(71,85,105,.22);color:var(--text3)}
.pill.warn{background:rgba(255,107,107,.14);color:var(--red)}
.pd{width:6px;height:6px;border-radius:50%;flex-shrink:0;
  background:currentColor;box-shadow:0 0 5px currentColor}

/* ── Battery Hero ── */
.hero{text-align:center;padding-bottom:4px}
.soc-ring{width:120px;height:120px;margin:0 auto 14px;position:relative}
.soc-ring svg{transform:rotate(-90deg)}
.trk{fill:none;stroke:#1e293b;stroke-width:8}
.bar{fill:none;stroke:var(--accent);stroke-width:8;stroke-linecap:round;
  transition:stroke-dashoffset .8s ease,stroke .5s}
.soc-val{position:absolute;inset:0;display:flex;flex-direction:column;
  align-items:center;justify-content:center}
.soc-num{font-size:2em;font-weight:700;line-height:1;
  font-variant-numeric:tabular-nums}
.soc-lbl{font-size:.6em;color:var(--text3);margin-top:3px;text-transform:uppercase}
.hg{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;text-align:center}
.hg .hv{font-size:1.1em;font-weight:600;font-variant-numeric:tabular-nums}
.hg .hl{font-size:.65em;color:var(--text3);margin-top:2px}

/* ── CAN stat grid ── */
.sg{display:grid;grid-template-columns:repeat(2,1fr);gap:8px}
.sb{background:var(--card2);border-radius:10px;padding:10px 12px}
.sb .sv{font-size:1.15em;font-weight:700;font-variant-numeric:tabular-nums}
.sb .sl{font-size:.64em;color:var(--text3);margin-top:2px}

/* ── Controls ── */
.btn-main{width:100%;padding:14px;border:none;border-radius:12px;
  font-size:.95em;font-weight:700;cursor:pointer;letter-spacing:.04em;
  transition:opacity .2s;margin-bottom:10px}
.btn-main:active{opacity:.75}
.btn-act{background:linear-gradient(135deg,var(--accent),var(--accent2));color:#000}
.btn-stop{background:rgba(255,107,107,.14);color:var(--red);
  border:1px solid rgba(255,107,107,.3)}
.sw{position:relative;width:44px;height:24px;flex-shrink:0}
.sw input{opacity:0;width:0;height:0}
.sl2{position:absolute;cursor:pointer;inset:0;background:#2a2a3e;
  border-radius:24px;transition:.3s}
.sl2:before{content:"";position:absolute;height:18px;width:18px;
  left:3px;bottom:3px;background:#555;border-radius:50%;transition:.3s}
input:checked+.sl2{background:var(--accent)}
input:checked+.sl2:before{transform:translateX(20px);background:#fff}

/* ── OTA firmware update ── */
.ota-file{display:none}.ota-progress{display:none;margin-top:12px}
.ota-track{background:var(--card2);border-radius:8px;height:10px;overflow:hidden}
.ota-bar{background:var(--accent);height:100%;width:0%;transition:width .2s}
.ota-status{text-align:center;margin-top:8px;font-size:.85em;color:var(--text2)}
.ota-bytes{text-align:center;margin-top:4px;font-size:.72em;color:var(--text3)}
.ota-info{margin-top:10px;padding:10px 12px;background:rgba(77,171,247,.07);
  border-radius:8px;border:1px solid rgba(77,171,247,.15);font-size:.72em;color:var(--text3);line-height:1.4}
.btn-blue{background:rgba(77,171,247,.14);color:var(--blue);border:1px solid rgba(77,171,247,.3)}
.btn-yellow{background:rgba(255,217,61,.14);color:var(--yellow);border:1px solid rgba(255,217,61,.3)}

/* ── Footer ── */
.foot{text-align:center;padding:16px 0 0;font-size:.64em;color:var(--text3)}

/* ── Auth panel ── */
.auth-panel,.confirm-panel{display:none;background:var(--card);border:1px solid var(--border);
  border-radius:12px;padding:16px;margin-bottom:12px}
.auth-panel.show,.confirm-panel.show{display:block}
.auth-box{width:100%;background:transparent;border:0;
  border-radius:0;padding:0;box-shadow:none}
.auth-box h3{font-size:1.15em;text-align:center;margin-bottom:12px}
.auth-msg{font-size:.82em;color:var(--text2);line-height:1.4;margin-bottom:14px}
.auth-field{display:block;font-size:.75em;color:var(--text2);margin:10px 0 5px}
.auth-input{width:100%;background:var(--card2);border:1px solid var(--border);
  color:var(--text);border-radius:8px;padding:11px;font-size:1em}
.auth-actions{display:flex;gap:10px;margin-top:16px}
.auth-actions button{flex:1;margin:0}
.log-info{font-size:.74em;color:var(--text2);line-height:1.45;margin-top:8px}
.log-actions{display:flex;gap:10px;margin-top:12px}
.log-actions button{flex:1;margin:0}
.log-filter{width:210px;max-width:60%;background:var(--card2);border:1px solid var(--border);
  color:var(--text);border-radius:6px;padding:6px 8px;font-size:.8em;text-align:right}
.log-filter::placeholder{color:var(--text3)}
.action-grid{display:grid;grid-template-columns:repeat(2,1fr);gap:8px;margin-top:10px}
.action-dot{display:inline-flex;align-items:center;gap:7px;color:var(--text2);font-size:.78em}
.blinkdot{width:10px;height:10px;border-radius:50%;background:var(--text3);box-shadow:none;transition:.15s}
.blinkdot.hit{background:var(--accent);box-shadow:0 0 10px var(--accent)}
.test-btn{padding:10px 8px;border-radius:10px;border:1px solid rgba(77,171,247,.3);
  background:rgba(77,171,247,.12);color:var(--blue);font-size:.74em;font-weight:700}
.test-btn:disabled{opacity:.4;color:var(--text3);border-color:var(--border);background:var(--card2)}
.metric{font-size:.95em;font-weight:700;font-variant-numeric:tabular-nums}
.metric small{font-size:.72em;color:var(--text3);font-weight:600}

/* 1.4.33-inspired information architecture: compact overview + section tabs.
   UI-only; existing control IDs/handlers and CAN/research behavior are unchanged. */
html{scroll-behavior:smooth}
.top-tabs{display:flex;gap:7px;overflow-x:auto;padding:4px 0 12px;scrollbar-width:none}
.top-tabs::-webkit-scrollbar{display:none}
.top-tabs a{flex:0 0 auto;text-decoration:none;color:var(--text2);font-size:.76em;font-weight:700;
  border:1px solid var(--border);background:var(--card2);padding:8px 11px;border-radius:10px}
.top-tabs a:hover,.top-tabs a:focus{color:var(--text);border-color:rgba(77,171,247,.55);outline:none}
.quick-card{padding:14px}
.quick-title{display:flex;align-items:center;justify-content:space-between;margin-bottom:10px}
.quick-title strong{font-size:.86em}
.quick-title span{font-size:.68em;color:var(--text3)}
.quick-grid{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}
.quick-item{background:var(--card2);border:1px solid var(--border);border-radius:10px;padding:10px 8px;text-align:center;min-width:0}
.quick-v{display:block;font-size:1.12em;font-weight:800;font-variant-numeric:tabular-nums;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.quick-l{display:block;margin-top:3px;font-size:.65em;color:var(--text3)}
.section-anchor{scroll-margin-top:10px}
@media(max-width:430px){.quick-grid{grid-template-columns:repeat(2,1fr)}}
.speed-panel .speed-live{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;margin:10px 0}
.speed-panel .speed-live>div{background:var(--card2);border:1px solid var(--border);border-radius:10px;padding:10px;text-align:center}
.speed-panel .speed-live b{display:block;font-size:1.2em;font-variant-numeric:tabular-nums}
.speed-panel .speed-live span{display:block;margin-top:3px;font-size:.65em;color:var(--text3)}
.target-grid{display:grid;grid-template-columns:repeat(5,1fr);gap:6px;margin-top:10px}
.target-cell{border:1px solid var(--border);background:var(--card2);border-radius:9px;padding:8px 4px;text-align:center;transition:.15s}
.target-cell strong{display:block;font-size:.82em}
.target-cell small{display:block;color:var(--text3);font-size:.62em;margin-top:2px}
.target-cell.active{border-color:var(--accent);box-shadow:0 0 0 1px rgba(55,212,150,.25);background:rgba(55,212,150,.08)}
.speed-advanced{margin-top:12px;border-top:1px solid var(--border);padding-top:10px}
.speed-advanced summary{cursor:pointer;color:var(--blue);font-size:.76em;font-weight:700}
.speed-notice{font-size:.7em;color:var(--text3);line-height:1.5;margin-top:10px}
.control-tier-tabs{display:grid;grid-template-columns:repeat(3,1fr);gap:6px;margin:4px 0 10px}
.control-tier-btn{border:1px solid var(--border);background:var(--card2);color:var(--text2);border-radius:9px;padding:9px 5px;
  font-size:.74em;font-weight:800;cursor:pointer}
.control-tier-btn.active{background:rgba(77,171,247,.14);border-color:rgba(77,171,247,.55);color:var(--blue)}
.control-tier-note{font-size:.68em;color:var(--text3);line-height:1.45;margin:0 0 8px}
.mode-quick{margin:2px 0 10px}
.mode-quick .hint{display:block;margin-top:5px;text-align:center}
@media(max-width:430px){
  .target-grid{grid-template-columns:repeat(3,1fr)}
  .speed-panel .speed-live{grid-template-columns:repeat(3,1fr)}
}
</style>
</head>
<body>
<div class="wrap">

<!-- Header -->
<div class="hdr">
  <h1>BSB Tesla FSD</h1>
  <div class="sub">ESP32 CAN 控制器 &middot; <span id="deviceHost">device.local</span></div>
  <div class="cdot" id="dot"></div>
</div>

<nav class="top-tabs" aria-label="Dashboard sections">
  <a href="#dashboardSection">DASHBOARD</a>
  <a href="#speedSection">速度策略</a>
  <a href="#perfTestSection">PERF TEST</a>
  <a href="#batterySection">BMS</a>
  <a href="#networkSection">网络 / 系统</a>
</nav>

<!-- Chip temperature: before FSD status, using the existing live telemetry IDs. -->
<div class="card chip-temperature" aria-label="ESP32 芯片温度监控" title="ESP32-S3 内部传感器，仅用于趋势监控">
  <span class="temp-reading"><span class="lbl">芯片温度</span><b id="chipTemp">--</b></span>
  <span class="temp-reading"><span class="lbl">最高</span><b id="chipTempMax">--</b></span>
  <span class="pill off" id="chipTempSt"><span class="pd"></span>--</span>
</div>
<div id="connErr" class="err">连接已断开 &mdash; 正在重试&hellip;</div>

<div id="authPanel" class="auth-panel">
  <div class="auth-box">
    <h3>Authentication Required</h3>
    <div class="auth-msg">Enter the admin username and the WiFi AP password.</div>
    <label class="auth-field" for="authUser">Username</label>
    <input id="authUser" class="auth-input" type="text" value="admin" autocomplete="username">
    <label class="auth-field" for="authPass">Password</label>
    <input id="authPass" class="auth-input" type="password" autocomplete="current-password">
    <div class="auth-actions">
      <button type="button" class="btn-main btn-stop" onclick="cancelAuth()">Cancel</button>
      <button type="button" class="btn-main btn-blue" onclick="submitAuth()">Sign In</button>
    </div>
  </div>
</div>

<div id="restartConfirmPanel" class="confirm-panel">
  <div class="auth-box">
    <h3>Restart device?</h3>
    <div class="auth-msg">The device will reboot immediately and the web connection will drop briefly.</div>
    <div class="auth-actions">
      <button type="button" class="btn-main btn-stop" onclick="cancelRestartConfirm()">No</button>
      <button type="button" class="btn-main btn-yellow" onclick="confirmRestart()">Yes</button>
    </div>
  </div>
</div>

<!-- OTA Warning -->
<div id="otaBanner" class="ota">&#9888;&#xFE0F; OTA UPDATE IN PROGRESS &mdash; CAN TX SUSPENDED</div>

<!-- Autopark Warning (#180) -->
<div id="autoparkBanner" class="ota">&#9888;&#xFE0F; IN-CAR AUTOPARK &mdash; CAN TX PAUSED</div>

<!-- Signal Map watchdog (#100) -->
<div id="sigmapWarn" class="warn14x"><div class="w-row"><div class="w-msg">
  <strong>&#9888;&#xFE0F; Signal Map DAS id not seen on this bus.</strong>
  The configured DAS id isn't arriving, so AP-state can't be read and the nag
  killer is paused. Set <b>DAS id 0</b> for auto, or fix the mapping / tap.
</div></div></div>

<!-- 2026.14.x Firmware Warning -->
<div id="warn14x" class="warn14x">
  <div class="w-row">
    <div class="w-msg">
      <strong>&#9888;&#xFE0F; 2026.14.x firmware enforcement active.</strong>
      Tesla added a preflight check in 2026.14.x that disables autosteer
      the moment any CAN frame touches <code>0x3FD</code>. Symptom on
      the dash: <em>"Autopilot turning off"</em> appears within a second
      of stalk engagement, then AP immediately disengages. Listen-Only
      mode is safe. <strong>AP-First</strong> (delay injection until AP is
      engaged) is on the Flipper build only right now — on the ESP32, engage
      AP from the stalk first, then turn on injection. Dismiss if you're on
      pre-14.x firmware.
    </div>
    <button class="w-dismiss" onclick="cmd('14x_warning',false)">Dismiss</button>
  </div>
</div>

<!-- 1.4.33-style compact dashboard: live summary only, no new control behavior -->
<div class="card quick-card section-anchor" id="dashboardSection">
  <div class="quick-title"><strong>驾驶状态概览</strong><span>LIVE / RESEARCH</span></div>
  <div class="quick-grid">
    <div class="quick-item"><span class="quick-v" id="qLimit">--</span><span class="quick-l">道路限速 km/h</span></div>
    <div class="quick-item"><span class="quick-v" id="qTarget">--</span><span class="quick-l">目标速度 km/h</span></div>
    <div class="quick-item"><span class="quick-v" id="qOffset">--</span><span class="quick-l">当前偏移 %</span></div>
    <div class="quick-item"><span class="quick-v" id="qHw">--</span><span class="quick-l">物理 HW 布局</span></div>
    <div class="quick-item"><span class="quick-v" id="qProto">--</span><span class="quick-l">FSD 协议</span></div>
    <div class="quick-item"><span class="quick-v" id="qTemp">--</span><span class="quick-l">芯片温度 °C</span></div>
  </div>
</div>

<!-- Common FSD controls: kept above status for quick access -->
<div class="card controls-section">
  <div class="card-head"><div class="icon ic-c">C</div><h2>FSD 常用控制</h2></div>
  <div class="mode-quick">
    <button id="btnMode" class="btn-main btn-act" onclick="toggleMode()">启用模拟计算</button>
    <span class="hint">研究分支：仅切换模拟计算；物理 CAN TX 保持禁用。</span>
  </div>
  <div class="row">
    <span class="lbl">FSD 激活</span>
    <label class="sw"><input type="checkbox" id="swFsdUnlock" onchange="cmd('fsd_unlock',this.checked)"><span class="sl2"></span></label>
  </div>
  <div class="row">
    <span class="lbl">NAG Killer</span>
    <label class="sw"><input type="checkbox" id="swNag" onchange="cmd('nag',this.checked)"><span class="sl2"></span></label>
  </div>
  <div class="row">
    <span class="lbl">Force FSD</span>
    <label class="sw"><input type="checkbox" id="swFsd" onchange="cmd('force_fsd',this.checked)"><span class="sl2"></span></label>
  </div>
  <div class="row">
    <span class="lbl">车辆硬件<br><span class="hint">建议自动识别；必要时可手动选择实际 HW3 / HW4。</span></span>
    <select id="selHwOverride" onchange="cmd('hw_override',parseInt(this.value,10))">
      <option value="0">自动识别</option>
      <option value="3">强制 HW4</option>
      <option value="2">强制 HW3</option>
      <option value="1">强制 Legacy</option>
    </select>
  </div>
  <div class="row">
    <span class="lbl">FSD 协议<br><span class="hint">与物理硬件分开设置。</span></span>
    <select id="selProtocol" onchange="cmd('fsd_protocol',parseInt(this.value,10))">
      <option value="0">自动</option>
      <option value="13">V13</option>
      <option value="14">V14</option>
    </select>
  </div>
  <div class="row" id="rowDriveStyle">
    <span class="lbl">驾驶风格<br><span class="hint">自动跟随原车，或手动锁定 0–4。</span></span>
    <select id="selDriveStyle" onchange="cmd('drive_style',parseInt(this.value,10))">
      <option value="0">自动（跟随原车）</option>
      <option value="1">轻松</option>
      <option value="2">普通</option>
      <option value="3">运动（2）</option>
      <option value="4">狂飙（3）</option>
      <option value="5">极限（4）</option>
    </select>
  </div>
</div>

<!-- FSD Status -->
<div class="card">
  <div class="card-head"><div class="icon ic-s">S</div><h2>FSD 状态</h2></div>
  <div class="row">
    <span class="lbl">AP 状态</span>
    <span class="pill off" id="fsdSt"><span class="pd"></span>--</span>
  </div>
  <div class="row">
    <span class="lbl">运行模式</span>
    <span class="pill off" id="opMode"><span class="pd"></span>--</span>
  </div>
  <div class="row">
    <span class="lbl">硬件</span>
    <span class="pill off" id="hwVer"><span class="pd"></span>--</span>
  </div>
  <div class="row">
    <span class="lbl">NAG Killer</span>
    <span class="pill off" id="nagSt"><span class="pd"></span>--</span>
  </div>
  <div class="row">
    <span class="lbl">车辆 CAN</span>
    <span class="pill off" id="canVeh"><span class="pd"></span>--</span>
  </div>
</div>

<!-- HTTP CAN Log: PERF TEST / research capture -->
<div class="card section-anchor" id="perfTestSection">
  <div class="card-head"><div class="icon ic-d">P</div><h2>PERF TEST / CAN 研究</h2>
    <span class="pill off" id="httpLogSt" style="margin-left:auto"><span class="pd"></span>Idle</span>
  </div>
  <div class="row">
    <span class="lbl">Filter IDs</span>
    <input id="httpLogFilter" class="log-filter" type="text" autocomplete="off" autocapitalize="off" spellcheck="false" value="0x399,0x3FD,0x3F8">
  </div>
  <div class="row">
    <span class="lbl">Buffered</span>
    <span id="httpLogBuf" style="font-size:.8em;color:var(--text2)">0 frames</span>
  </div>
  <div class="row">
    <span class="lbl">Dropped</span>
    <span id="httpLogDrop" style="font-size:.8em;color:var(--text2)">0 frames</span>
  </div>
  <div class="row">
    <span class="lbl">Filtered</span>
    <span id="httpLogFiltered" style="font-size:.8em;color:var(--text2)">0 frames</span>
  </div>
  <div id="httpLogInfo" class="log-info">Ready to collect 0x399 / 0x3FD / 0x3F8 in this browser.</div>
  <div style="margin:10px 0;padding:10px;border:1px solid var(--border);border-radius:10px;background:var(--card2)">
    <div style="display:grid;grid-template-columns:1fr 1fr;gap:8px 12px;font-size:.82em">
      <div><span class="lbl">399 候选限速</span><br><b id="diag399Limit" style="font-size:1.15em">--</b> km/h</div>
      <div><span class="lbl">399 Raw</span><br><b id="diag399Raw" style="font-size:1.15em">--</b></div>
      <div><span class="lbl">3FD Mux</span><br><b id="diag3fdMux" style="font-size:1.15em">--</b></div>
      <div><span class="lbl">输入 Offset</span><br><b id="diag3fdOffset" style="font-size:1.15em">--</b>%</div>
      <div><span class="lbl">输入 Profile</span><br><b id="diag3fdProfile" style="font-size:1.15em">--</b></div>
      <div><span class="lbl">3F8 Follow</span><br><b id="diag3f8Follow" style="font-size:1.15em">--</b></div>
    </div>
    <div class="hint" style="margin-top:8px">测试版：关键速度偏移诊断常驻显示，方便边抓包边观察。</div>
  </div>
  <button id="btnHttpLog" type="button" class="btn-main btn-blue" onclick="toggleHttpLog()">STREAM LOG AND SAVE</button>
</div>

<!-- Battery / BMS diagnostics -->
<details class="config-section" id="batterySection">
  <summary><div class="icon ic-b">B</div><div class="card-head" style="margin:0"><h2>车辆电池 / BMS（诊断）</h2></div><span id="batterySummary" style="margin-left:auto;font-size:.72em;color:var(--text3)">未检测</span></summary>
  <div class="config-body">
  <div class="card" style="margin-bottom:0">
  <div class="row">
    <span class="lbl">BMS 状态</span>
    <span class="pill off" id="bmsSt"><span class="pd"></span>Waiting Frames</span>
  </div>
  <div class="row">
    <span class="lbl">BMS 帧</span>
    <span id="bmsFrames" style="font-size:.8em;color:var(--text2)">HV:0 SOC:0 TH:0</span>
  </div>
  <div class="hero">
    <div class="soc-ring">
      <svg viewBox="0 0 120 120" width="120" height="120">
        <circle class="trk" cx="60" cy="60" r="52"/>
        <circle class="bar" id="socBar" cx="60" cy="60" r="52"
          stroke-dasharray="326.73" stroke-dashoffset="326.73"/>
      </svg>
      <div class="soc-val">
        <span class="soc-num" id="bSoc">--</span>
        <span class="soc-lbl">SOC</span>
      </div>
    </div>
    <div class="hg">
      <div><div class="hv" id="bVolt">--</div><div class="hl">电压</div></div>
      <div><div class="hv" id="bCurr">--</div><div class="hl">电流</div></div>
      <div><div class="hv" id="bTemp">--</div><div class="hl">温度</div></div>
    </div>
  </div>
  </div>
  </div>
</details>

<!-- CAN Stats -->
<div class="card">
  <div class="card-head"><div class="icon ic-d">C</div><h2>CAN 总线（只监听；输出为 mock）</h2></div>
  <div class="sg">
    <div class="sb"><div class="sv" id="rxCnt">0</div><div class="sl">接收帧</div></div>
    <div class="sb"><div class="sv" id="txCnt">0</div><div class="sl">物理发送帧（始终 0）</div></div>
    <div class="sb"><div class="sv" id="crcErr">0</div><div class="sl">CAN 错误</div><div class="sl" id="crcSplit">RX&nbsp;missed&nbsp;0 &middot; bus&nbsp;0 &middot; TX&nbsp;fail&nbsp;0</div></div>
    <div class="sb"><div class="sv" id="fps">0.0</div><div class="sl">帧/秒</div></div>
  </div>
</div>

<!-- 1.4.33-inspired speed strategy: simple first, advanced details folded -->
<div class="card speed-panel section-anchor" id="speedSection">
  <div class="card-head"><div class="icon ic-s">S</div><h2>速度策略</h2></div>
  <div class="row">
    <span class="lbl">模式</span>
    <select id="offsetMode" onchange="cmd('hw3_offset_mode',parseInt(this.value,10))">
      <option value="0">手动偏移</option>
      <option value="1">自动目标</option>
      <option value="2">自定义分区</option>
    </select>
  </div>
  <div class="speed-live">
    <div><b id="spdLimitNow">--</b><span>当前限速 km/h</span></div>
    <div><b id="spdTargetNow">--</b><span>目标速度 km/h</span></div>
    <div><b id="spdOffNow">--</b><span>当前偏移 %</span></div>
  </div>
  <div class="hint">自动目标参考表（按当前研究源码算法）</div>
  <div class="target-grid" id="targetGrid">
    <div class="target-cell" data-limit="30"><strong>30 → 49</strong><small>+63%</small></div>
    <div class="target-cell" data-limit="40"><strong>40 → 60</strong><small>+50%</small></div>
    <div class="target-cell" data-limit="50"><strong>50 → 75</strong><small>+50%</small></div>
    <div class="target-cell" data-limit="60"><strong>60 → 90</strong><small>+50%</small></div>
    <div class="target-cell" data-limit="70"><strong>70 → 91</strong><small>+30%</small></div>
    <div class="target-cell" data-limit="80"><strong>80 → 104</strong><small>+30%</small></div>
    <div class="target-cell" data-limit="90"><strong>90 → 117</strong><small>+30%</small></div>
    <div class="target-cell" data-limit="100"><strong>100 → 120</strong><small>+20%</small></div>
    <div class="target-cell" data-limit="110"><strong>110 → 132</strong><small>+20%</small></div>
    <div class="target-cell" data-limit="120"><strong>120 → 132</strong><small>+10%</small></div>
  </div>
  <details class="speed-advanced">
    <summary>高级设置 / 研究参数</summary>
    <div class="row"><span class="lbl">手动偏移 %</span><input id="manualOffset" type="number" min="0" max="63" onchange="speedNum('hw3_manual_offset',this,0,63)"></div>
    <div class="speed-bands">
      <label class="speed-band">≤50 km/h <input id="cp0" type="number" min="0" max="63" onchange="speedNum('hw3_cp0',this,0,63)">%</label>
      <label class="speed-band">≤70 km/h <input id="cp1" type="number" min="0" max="63" onchange="speedNum('hw3_cp1',this,0,63)">%</label>
      <label class="speed-band">≤100 km/h <input id="cp2" type="number" min="0" max="63" onchange="speedNum('hw3_cp2',this,0,63)">%</label>
      <label class="speed-band">&gt;100 km/h <input id="cp3" type="number" min="0" max="63" onchange="speedNum('hw3_cp3',this,0,63)">%</label>
    </div>
    <p class="hint">15–45 km/h 使用源码固定规则，优先于手动/自定义。自动：50–60 +50%，70–90 +30%，100–110 +20%，120 +10%。</p>
    <p class="hint">目标 cap：15–40→60，45→67，50→75，55→82，60→90，70→91，80→104，90→117，100→120，110–120→132 km/h。</p>
    <p class="hint">下降按 3 km/h/s 平滑；至少 0.5 秒更新一次。偏移字段限制 0–63%。</p>
    <div>目标偏移 <b id="spdOffTarget">--</b>%</div>
    <div id="researchValues">等待回放</div>
  </details>
  <div class="speed-notice">界面参考 1.4.33 的“目标速度优先”交互，但数值与编码仍采用当前研究分支的源码模型。</div>
</div>

<!-- Controls -->
<div class="card controls-section">
  <div class="card-head"><div class="icon ic-c">C</div><h2>FSD 更多设置</h2></div>
  <div class="control-tier-tabs" role="tablist" aria-label="FSD control level">
    <button type="button" class="control-tier-btn active" id="tierRegular" onclick="setControlTier('regular')">常规</button>
    <button type="button" class="control-tier-btn" id="tierAdvanced" onclick="setControlTier('advanced')">高级</button>
    <button type="button" class="control-tier-btn" id="tierExperimental" onclick="setControlTier('experimental')">实验</button>
  </div>
  <div class="control-tier-note" id="tierNote">日常设置。复杂兼容选项和未经充分验证的功能分别放在“高级 / 实验”。</div>
  <div class="controls-body" id="tierControls">
  <div class="row" data-tier="advanced">
    <span class="lbl">物理 TX（台架预留）<br><span class="hint">当前研究固件保持禁用；此项仅作为今后台架测试入口。</span></span>
    <label class="sw"><input type="checkbox" id="swBenchTxReserved" disabled><span class="sl2"></span></label>
  </div>
  <div class="row" data-tier="advanced">
    <span class="lbl">忽略 OTA</span>
    <label class="sw"><input type="checkbox" id="swIgnoreOta" onchange="cmd('ignore_ota',this.checked)"><span class="sl2"></span></label>
  </div>
  <div class="row" data-tier="regular">
    <span class="lbl">连续 AP</span>
    <label class="sw"><input type="checkbox" id="swContinuousAp" onchange="cmd('continuous_ap',this.checked)"><span class="sl2"></span></label>
  </div>
  <div class="row" data-tier="advanced">
    <span class="lbl">AP-First (14.x)</span>
    <label class="sw"><input type="checkbox" id="swApFirst" onchange="cmd('ap_first',this.checked)"><span class="sl2"></span></label>
  </div>
  <div class="row" data-tier="experimental">
    <span class="lbl">Instant Engage (exp.)</span>
    <label class="sw"><input type="checkbox" id="swApFe" onchange="cmd('ap_first_edge',this.checked)"><span class="sl2"></span></label>
  </div>
  <div class="row" data-tier="experimental">
    <span class="lbl">Minimal Inject (exp.)</span>
    <label class="sw"><input type="checkbox" id="swApMi" onchange="cmd('ap_first_minimal',this.checked)"><span class="sl2"></span></label>
  </div>
  <div class="row" data-tier="experimental">
    <span class="lbl">Nag EPAS-faithful (14.x, exp.)</span>
    <label class="sw"><input type="checkbox" id="swNagF" onchange="cmd('nag_faithful',this.checked)"><span class="sl2"></span></label>
  </div>
  <div class="row" data-tier="experimental">
    <span class="lbl">Soft Engage (14.x, exp.)</span>
    <label class="sw"><input type="checkbox" id="swSoft" onchange="cmd('soft_engage',this.checked)"><span class="sl2"></span></label>
  </div>
  <div class="row" data-tier="experimental">
    <span class="lbl">Nag Burst (14.x, exp.)</span>
    <label class="sw"><input type="checkbox" id="swNagB" onchange="cmd('nag_burst',this.checked)"><span class="sl2"></span></label>
  </div>
  <div class="row" data-tier="experimental">
    <span class="lbl">Abort Guard (14.x, exp.)</span>
    <label class="sw"><input type="checkbox" id="swAbrt" onchange="cmd('abort_guard',this.checked)"><span class="sl2"></span></label>
  </div>
  <div class="row" data-tier="regular">
    <span class="lbl">BMS 显示</span>
    <label class="sw"><input type="checkbox" id="swBms" onchange="cmd('bms',this.checked)"><span class="sl2"></span></label>
  </div>
  <div class="row" data-tier="advanced">
    <span class="lbl">中国模式</span>
    <label class="sw"><input type="checkbox" id="swChina" onchange="cmd('china_mode',this.checked)"><span class="sl2"></span></label>
  </div>
  <div class="row" id="rowChime" data-tier="regular">
    <span class="lbl">关闭限速提示音</span>
    <label class="sw"><input type="checkbox" id="swChime" onchange="cmd('suppress_speed_chime',this.checked)"><span class="sl2"></span></label>
  </div>
  <div class="row" data-tier="advanced">
    <span class="lbl">TLSSC Restore</span>
    <label class="sw"><input type="checkbox" id="swTlssc" onchange="cmd('tlssc_restore',this.checked)"><span class="sl2"></span></label>
  </div>
  <div class="row" data-tier="advanced">
    <span class="lbl">Summon EU Unlock</span>
    <label class="sw"><input type="checkbox" id="swSummon" onchange="cmd('summon_unlock',this.checked)"><span class="sl2"></span></label>
  </div>
  <div class="row" data-tier="advanced">
    <span class="lbl">Continue on Green<br><small style="color:var(--muted)">pairs with TLSSC</small></span>
    <label class="sw"><input type="checkbox" id="swCog" onchange="cmd('continue_on_green',this.checked)"><span class="sl2"></span></label>
  </div>
  <div class="row" data-tier="advanced">
    <span class="lbl">TLSSC bit38</span>
    <label class="sw"><input type="checkbox" id="swTlssc38" onchange="cmd('assist_tlssc_bit38',this.checked)"><span class="sl2"></span></label>
  </div>
  <div class="row" data-tier="advanced">
    <span class="lbl">Right-Hand Drive (RHD)<br><small style="color:var(--red)">RHD markets only — do NOT enable while driving on the right.</small></span>
    <label class="sw"><input type="checkbox" id="swRhd" onchange="cmd('assist_rhd_override',this.checked)"><span class="sl2"></span></label>
  </div>
  <div class="row" data-tier="experimental">
    <span class="lbl">Telemetry Off (experimental)<br><small style="color:var(--muted)">Experimental &amp; unverified — clears reachable telemetry flags only (not the Vehicle-bus ECU log-upload). Does NOT guarantee reduced detection.</small></span>
    <label class="sw"><input type="checkbox" id="swTelOff" onchange="cmd('assist_telemetry_off',this.checked)"><span class="sl2"></span></label>
  </div>
  <div class="row" data-tier="experimental">
    <span class="lbl">AP Branch/Tier (experimental)<br><small style="color:var(--muted)">Experimental &amp; non-persistent — injects a UI branch/tier hint only, reverts when injection stops; unverified and may be a ban signal. Off by default.</small></span>
    <select id="selApmv3" onchange="cmd('apmv3_branch',parseInt(this.value,10))">
      <option value="255">Off</option>
      <option value="0">Live</option>
      <option value="1">Stage</option>
      <option value="2">Dev</option>
      <option value="3">Stage2</option>
      <option value="4">EAP</option>
      <option value="5">Demo</option>
    </select>
  </div>
  <div class="row" data-tier="experimental" style="display:block">
    <div style="display:flex;align-items:center;justify-content:space-between">
      <span class="lbl">Track Mode (experimental)<br><small style="color:var(--muted)">Experimental &mdash; Vehicle-bus; not car-validated. Defaults to rear-biased (rotation 100) + 30% stability &mdash; fun with a safety margin. Raise stability for stock feel.</small></span>
      <label class="sw"><input type="checkbox" id="swTrkMode" onchange="cmd('track_mode_inject',this.checked)"><span class="sl2"></span></label>
    </div>
    <div style="margin-top:8px">
      <label style="display:flex;justify-content:space-between;font-size:12px;color:var(--muted)"><span>Handling Balance <small>(stable &rarr; rotation)</small></span><span id="trkRotV">100</span></label>
      <input type="range" id="trkRot" min="0" max="100" style="width:100%" oninput="document.getElementById('trkRotV').textContent=this.value" onchange="cmd('track_rotation_pct',parseInt(this.value,10))">
    </div>
    <div style="margin-top:6px">
      <label style="display:flex;justify-content:space-between;font-size:12px;color:var(--muted)"><span>Stability Assist</span><span id="trkStabV">30</span></label>
      <input type="range" id="trkStab" min="0" max="100" style="width:100%" oninput="document.getElementById('trkStabV').textContent=this.value" onchange="cmd('track_stability_pct',parseInt(this.value,10))">
    </div>
    <div class="row" style="padding:6px 0 0">
      <span class="lbl">Post-drive Cooling</span>
      <label class="sw"><input type="checkbox" id="swTrkPC" onchange="cmd('track_post_cooling',this.checked)"><span class="sl2"></span></label>
    </div>
    <div class="row" style="padding:0">
      <span class="lbl">Compressor Overclock<br><small style="color:var(--muted)">max cooling</small></span>
      <label class="sw"><input type="checkbox" id="swTrkCO" onchange="cmd('track_cmp_overclock',this.checked)"><span class="sl2"></span></label>
    </div>
  </div>
  <div class="row" data-tier="advanced" style="display:block">
    <div id="pmSuggest" style="display:none;margin:0 0 8px;padding:8px 10px;border:1px solid var(--accent);border-radius:6px;background:var(--card2)">
      <div style="font-size:12px;color:var(--text)">Looks like variant <b id="pmName">?</b> &mdash; the standard parser can't read AP-state on this bus.</div>
      <button type="button" id="pmApply" onclick="pmApply()" style="margin-top:6px;background:var(--accent);color:#000;border:0;padding:6px 12px;border-radius:4px;cursor:pointer">Apply this profile</button>
    </div>
    <details>
      <summary class="lbl" style="cursor:pointer">Signal Map (advanced, 14.x)</summary>
      <div style="font-size:11px;color:var(--muted);margin:6px 0 8px">Override where the nag killer reads AP-state / hands-on / steering. Leave DAS id <b>0</b> for auto-detect. byte 0-7, shift 0-7, mask hex.</div>
      <div style="display:grid;grid-template-columns:auto auto;gap:6px 10px;align-items:center;font-size:12px">
        <span>DAS id (0x..)</span><input id="cgDid" placeholder="0">
        <span>AP-state byte/sh/mask</span><span><input id="cgApB" class="cgn"><input id="cgApS" class="cgn"><input id="cgApM" class="cgn" placeholder="0xF"></span>
        <span>Hands-on byte/sh/mask</span><span><input id="cgHoB" class="cgn"><input id="cgHoS" class="cgn"><input id="cgHoM" class="cgn" placeholder="0xF"></span>
        <span>Steer id (0x..)</span><input id="cgSid" placeholder="0">
        <span>Steer hi/lo byte</span><span><input id="cgSHi" class="cgn"><input id="cgSLo" class="cgn"></span>
      </div>
      <button onclick="saveSigCfg()" style="margin-top:8px;background:var(--accent);color:#000;border:0;padding:6px 12px;border-radius:4px;cursor:pointer">Save mapping</button>
    </details>
  </div>
)rawliteral"
#if defined(BOARD_TTGO_DISPLAY)
R"rawliteral(
  <div class="row" data-tier="advanced">
    <span class="lbl">TTGO Display</span>
    <label class="sw"><input type="checkbox" id="swDisp" onchange="cmd('disp',this.checked)"><span class="sl2"></span></label>
  </div>
  <div class="row" data-tier="advanced">
    <span class="lbl">Display Brightness (%)</span>
    <input type="number" id="dispBr" min="0" max="100" style="width:60px;background:var(--card2);border:1px solid var(--border);color:var(--text);padding:4px;border-radius:4px;text-align:right" onchange="cmd('disp_br',parseInt(this.value))">
  </div>
  <div class="row" data-tier="advanced">
    <span class="lbl">Display Timeout (s)</span>
    <input type="number" id="dispTo" min="0" max="3600" style="width:60px;background:var(--card2);border:1px solid var(--border);color:var(--text);padding:4px;border-radius:4px;text-align:right" onchange="cmd('disp_to',parseInt(this.value))">
  </div>
)rawliteral"
#endif
R"rawliteral(
  <div class="row" data-tier="regular">
    <span class="lbl">CAN 记录</span>
    <label class="sw"><input type="checkbox" id="swDump" onchange="cmd('dump',this.checked)"><span class="sl2"></span></label>
  </div>
)rawliteral"
#if defined(BOARD_LILYGO)
R"rawliteral(
  <div class="row" data-tier="advanced">
    <span class="lbl">Deep Sleep (sec)</span>
    <input type="number" id="numSleep" min="10" max="3600" style="width:60px;background:var(--card2);border:1px solid var(--border);color:var(--text);padding:4px;border-radius:4px;text-align:right" onchange="cmd('sleep',parseInt(this.value)*1000)">
  </div>
)rawliteral"
#endif
R"rawliteral(
  </div>
</div>

<!-- Tap capability checker (#125) -->
<div class="card">
  <div class="card-head"><div class="icon ic-d">T</div><h2>Tap Check</h2>
    <span id="capSt" class="pill off" style="margin-left:auto"><span class="pd"></span>Idle</span></div>
  <div class="log-info" style="margin-bottom:8px">
    Listens a few seconds and reports whether each feature can work on the bus this
    device is tapped into. Pure read-only &mdash; nothing is transmitted.
  </div>
  <div id="capBody"></div>
  <button id="capBtn" type="button" class="btn-main btn-blue" onclick="cmd('capability_recheck',true)" style="margin-top:8px">RE-CHECK</button>
</div>

<!-- Black-box incident recorder (#124) -->
<div class="card">
  <div class="card-head"><div class="icon ic-d">R</div><h2>Black-box</h2>
    <span id="bbBadge" class="pill on" style="display:none;margin-left:auto">NEW</span></div>
  <div id="bbNote" class="log-info" style="margin-bottom:10px;display:none">
    Records the key diagnostic CAN IDs around anomalies (aborts, bus-off, manual
    marks) to the device only &mdash; never uploaded. Use the toggle below to enable or disable.
    <button type="button" onclick="bbDismiss()" style="margin-left:6px;background:var(--card2);border:1px solid var(--border);color:var(--text);padding:2px 8px;border-radius:4px;cursor:pointer">Got it</button>
  </div>
  <div class="row">
    <span class="lbl">Auto-record</span>
    <label class="sw"><input type="checkbox" id="swBlackbox" onchange="cmd('blackbox_enable',this.checked)"><span class="sl2"></span></label>
  </div>
  <div class="row">
    <span class="lbl">Status</span>
    <span class="pill off" id="bbSt"><span class="pd"></span>Idle</span>
  </div>
  <div class="row">
    <span class="lbl">Storage</span>
    <span id="bbStore" style="font-size:.78em;color:var(--text2)">--</span>
  </div>
  <div id="bbVolatileWarn" class="log-info" style="display:none;margin:8px 0;color:var(--yellow)">
    &#9888;&#xFE0F; Volatile storage &mdash; download events before power-off; they are lost on reboot.
  </div>
  <button id="bbMark" type="button" class="btn-main btn-blue" onclick="cmd('blackbox_mark',true)" style="margin:8px 0">&#9873; MARK NOW</button>
  <div id="bbList" style="margin-top:4px"></div>
  <button id="bbDelAll" type="button" class="btn-main btn-stop" onclick="bbDeleteAll()" style="margin-top:10px;display:none">DELETE ALL EVENTS</button>
</div>

<!-- Administration -->
<details class="config-section section-anchor" id="networkSection">
  <summary><div class="icon ic-c">A</div><div class="card-head" style="margin:0"><h2>网络 / 系统管理</h2></div></summary>
  <div class="config-body">



<!-- WiFi Config -->
<div class="card">
  <div class="card-head"><div class="icon ic-c">W</div><h2>WiFi 设置</h2></div>
  <div class="log-info" style="margin-bottom:10px">
    The device starts its own access point by default. Optionally set a network below; when a network name is set, the device tries to connect to it on boot and starts its own access point if it cannot connect.
  </div>
  <div class="row">
    <span class="lbl">Access Point</span>
    <span style="font-size:.72em;color:var(--text3)">default</span>
  </div>
  <div class="row">
    <span class="lbl">SSID</span>
    <input type="text" id="wifiSsid" maxlength="32" style="width:140px;background:var(--card2);border:1px solid var(--border);color:var(--text);padding:4px;border-radius:4px;text-align:right">
  </div>
  <div class="row">
    <span class="lbl">密码</span>
    <input type="password" id="wifiPass" maxlength="64" style="width:140px;background:var(--card2);border:1px solid var(--border);color:var(--text);padding:4px;border-radius:4px;text-align:right">
  </div>
  <div class="row">
    <span class="lbl">隐藏热点</span>
    <label class="sw"><input type="checkbox" id="swWifiHid"><span class="sl2"></span></label>
  </div>
  <div class="row">
    <span class="lbl">Connect to WiFi</span>
    <span style="font-size:.72em;color:var(--text3)">optional</span>
  </div>
  <div class="row">
    <span class="lbl">网络名称</span>
    <input type="text" id="wifiStaSsid" maxlength="32" style="width:140px;background:var(--card2);border:1px solid var(--border);color:var(--text);padding:4px;border-radius:4px;text-align:right">
  </div>
  <div class="row">
    <span class="lbl">网络密码</span>
    <input type="password" id="wifiStaPass" maxlength="64" style="width:140px;background:var(--card2);border:1px solid var(--border);color:var(--text);padding:4px;border-radius:4px;text-align:right">
  </div>
  <button class="btn-main btn-stop" onclick="saveWifi()" style="margin-top:12px">保存并重启 WiFi</button>
</div>

<!-- OTA Update -->
<div class="card">
  <div class="card-head"><div class="icon ic-c">U</div><h2>OTA 固件更新</h2></div>
  <div style="font-size:.75em;color:var(--text3);margin-bottom:12px;line-height:1.5">
    Upload a .bin firmware file. Device will reboot after a successful update.
  </div>
  <form id="otaForm" enctype="multipart/form-data" style="margin:0">
    <input type="file" id="otaFile" class="ota-file" accept=".bin" onchange="uploadFirmware()">
    <button type="button" class="btn-main btn-blue" id="otaSelectBtn" onclick="selectFirmware(this)">
      选择固件 (.bin)
    </button>
  </form>
  <div id="otaProgress" class="ota-progress">
    <div class="ota-track"><div id="otaBar" class="ota-bar"></div></div>
    <div id="otaStatus" class="ota-status">Preparing...</div>
    <div id="otaBytes" class="ota-bytes"></div>
  </div>
  <div id="otaRollbackInfo" class="ota-info">
    <b style="color:var(--blue)">Partition Safety</b><br>
    OTA writes to the next app partition when available. Keep USB reflashing available as a recovery path.
  </div>
</div>

<!-- SD Card -->
<div class="card">
  <div class="card-head"><div class="icon ic-d">S</div><h2>SD Card</h2></div>
  <div class="row">
    <span class="lbl">Dump Status</span>
    <span class="pill off" id="dumpSt"><span class="pd"></span>Idle</span>
  </div>
  <button id="btnFmt" class="btn-main btn-stop" onclick="sdFormat()" style="margin-top:8px">FORMAT SD CARD</button>
  <div id="fmtOut" style="font-size:.75em;color:var(--text2);margin-top:8px;display:none"></div>
</div>

<!-- Device Info -->
<div class="card">
  <div class="card-head"><div class="icon ic-d">D</div><h2>设备</h2></div>
  <div class="row">
    <span class="lbl">固件版本</span>
    <span id="fwBuild" style="font-size:.8em;color:var(--text2)">--</span>
  </div>
  <div class="row">
    <span class="lbl">运行时间</span>
    <span id="uptime" style="font-variant-numeric:tabular-nums">--</span>
  </div>
  <div class="row">
    <span class="lbl">WiFi 客户端</span>
    <span id="wifiCl">--</span>
  </div>
  <div class="row">
    <span class="lbl">OTA 分区</span>
    <span id="otaPartInfo" style="font-size:.78em;color:var(--text2)">--</span>
  </div>
  <button class="btn-main btn-yellow" onclick="restartDevice(this)" style="margin-top:12px">重启设备</button>
</div>

  </div>
</details>

<div class="foot">Tesla FSD ESP32 &middot;
)rawliteral"
#if defined(BOARD_TTGO_DISPLAY)
R"rawliteral( TTGO T-Display + MCP2515)rawliteral"
#else
R"rawliteral( M5Stack ATOM Lite + ATOMIC CAN Base)rawliteral"
#endif
R"rawliteral(</div>
<div class="foot">BSB Tesla FSD 中文增强版 &middot; <a href="https://github.com/carterbonny-star/bsb-tesla-fsd" target="_blank" rel="noopener">GitHub 项目主页</a></div>
<div class="foot">基于 flipper-tesla-fsd 开源项目</div>
</div><!-- /wrap -->

<script>
var ws,rt,busy=0,wifiOnce=false,authHeader='',authAction=null,restartAnchor=null;
var httpLogAbort=null,httpLogReader=null,httpLogParts=[],httpLogBytes=0,httpLogStarted=0,httpLogRunning=false;
var httpLogName='',httpLogReady=false,httpLogSaveUrl='';
var httpLogAllowed=true;
var HW=['Unknown','Legacy','HW3','HW4'];
var CIRC=326.73;
document.getElementById('deviceHost').textContent=location.host||location.hostname||'192.168.4.1';

function initWifi(d){
  if(wifiOnce)return;
  wifiOnce=true;
  document.getElementById('wifiSsid').value=d.wifi_ssid||'';
  document.getElementById('wifiPass').value=d.wifi_pass||'';
  document.getElementById('swWifiHid').checked=!!d.wifi_hidden;
  document.getElementById('wifiStaSsid').value=d.wifi_sta_ssid||'';
  document.getElementById('wifiStaPass').value=d.wifi_sta_pass||'';
}

function fmt(s){
  var h=Math.floor(s/3600),m=Math.floor((s%3600)/60),sc=s%60;
  return h+':'+(m<10?'0':'')+m+':'+(sc<10?'0':'')+sc;
}
function socCol(p){return p>60?'var(--accent)':p>30?'var(--yellow)':'var(--red)';}
function pill(id,on,txt,warnClass){
  var e=document.getElementById(id);
  e.className='pill '+(warnClass||''+(on?'on':'off'));
  e.innerHTML='<span class="pd"></span>'+txt;
}
function dot(id,on){
  var e=document.getElementById(id);
  if(e)e.className='blinkdot'+(on?' hit':'');
}
function speedText(v,seen,source){
  if(!seen)return '--';
  var src=source?'<small> '+source+'</small>':'';
  return Math.round(v)+' km/h'+src;
}
var currentControlTier='regular';
function setControlTier(tier){
  currentControlTier=tier;
  var rows=document.querySelectorAll('#tierControls [data-tier]');
  for(var i=0;i<rows.length;i++) rows[i].style.display=(rows[i].getAttribute('data-tier')===tier)?'':'none';
  var ids={regular:'tierRegular',advanced:'tierAdvanced',experimental:'tierExperimental'};
  Object.keys(ids).forEach(function(k){var b=document.getElementById(ids[k]);if(b)b.classList.toggle('active',k===tier);});
  var note=document.getElementById('tierNote');
  if(note){
    note.textContent=tier==='regular'?'日常设置。复杂兼容选项和未经充分验证的功能分别放在“高级 / 实验”。':
      tier==='advanced'?'兼容性与地区/车型相关设置；不确定用途时保持默认。':
      '实验功能：用于研究验证，默认保持关闭。';
  }
}
document.addEventListener('DOMContentLoaded',function(){setControlTier('regular');});

function updateControlsSummary(d){
  var e=document.getElementById('controlsSummary');
  if(!e)return;
  var items=[];
  if(d.op_mode===1)items.push('Active');
  if(d.ignore_ota)items.push('Ignore OTA');
  if(d.fsd_unlock)items.push('FSD Unlock');
  if(d.nag_killer)items.push('NAG Killer');
  if(d.continuous_ap)items.push('Continuous AP');
  if(d.ap_first)items.push('AP-First');
  if(d.bms_output)items.push('BMS');
  if(d.force_fsd)items.push('Force FSD');
  if(d.china_mode)items.push('China');
  if(d.isa_speed_enabled&&d.suppress_speed_chime)items.push('Chime');
  if(d.tlssc_restore)items.push('TLSSC');
  if(d.assist_tlssc_bit38)items.push('TLSSC bit38');
  if(d.display_enabled)items.push('Display');
  if(d.can_dump)items.push('CAN Dump');
  e.textContent=items.length?items.join(', '):'Expand to setup';
  e.title=e.textContent;
}
// ── Black-box incident recorder (#124) ──
var bbCaptures=-1,bbNew=false,bbNoteDismissed=false;
try{bbNoteDismissed=localStorage.getItem('bbNote')==='1'}catch(e){}
function bbEsc(s){return String(s==null?'':s).replace(/[<>&"]/g,function(c){return{'<':'&lt;','>':'&gt;','&':'&amp;','"':'&quot;'}[c]})}
function bbDismiss(){bbNoteDismissed=true;try{localStorage.setItem('bbNote','1')}catch(e){}var n=document.getElementById('bbNote');if(n)n.style.display='none';}
function bbSeen(){bbNew=false;var bd=document.getElementById('bbBadge');if(bd)bd.style.display='none';}
function bbSync(d){
  var b=d.blackbox;if(!b)return;
  var sw=document.getElementById('swBlackbox');if(sw&&sw!==document.activeElement)sw.checked=!!b.enabled;
  pill('bbSt',b.enabled,b.armed?'Capturing':(b.enabled?'Armed':'Off'));
  var st=document.getElementById('bbStore');
  if(st)st.textContent=(b.backend||'?')+' · '+(b.psram?'PSRAM':'internal')+' · '+((b.cap||0).toLocaleString())+' frames · '+(b.events||0)+' saved';
  var vw=document.getElementById('bbVolatileWarn');if(vw)vw.style.display=b.volatile?'block':'none';
  var note=document.getElementById('bbNote');if(note)note.style.display=bbNoteDismissed?'none':'block';
  var da=document.getElementById('bbDelAll');if(da)da.style.display=(b.events>0)?'block':'none';
  if(bbCaptures<0){bbCaptures=b.captures;bbRefreshList();}
  else if(b.captures!==bbCaptures){bbCaptures=b.captures;bbNew=true;bbRefreshList();}
  var bd=document.getElementById('bbBadge');if(bd)bd.style.display=bbNew?'inline-block':'none';
}
function bbRefreshList(){
  fetch('/blackbox/list').then(function(r){return r.json()}).then(function(a){
    var el=document.getElementById('bbList');if(!el)return;
    if(!a||!a.length){el.innerHTML='<div style="font-size:.78em;color:var(--text3);padding:6px 0">No events recorded yet.</div>';return;}
    a.sort(function(x,y){return (y.name||'').localeCompare(x.name||'')});
    var h='';
    a.forEach(function(ev){
      var s=ev.summary||{},nm=encodeURIComponent(ev.name);
      h+='<div style="border-top:1px solid var(--border);padding:8px 0">';
      h+='<div style="font-size:.8em;color:var(--text);font-family:monospace">'+bbEsc(s.detail||s.trigger||'event')+'</div>';
      h+='<div style="font-size:.72em;color:var(--text3);margin:2px 0">'+bbEsc(s.hw||'')+' · '+(s.frames||0)+' frames · '+bbEsc((s.buses&&s.buses.dual_can)?'dual-CAN':'single')+'</div>';
      h+='<div style="display:flex;gap:6px;margin-top:4px">';
      h+='<a class="btn-main btn-blue" style="padding:4px 10px;font-size:.7em;flex:0" href="/blackbox/get?name='+nm+'&type=log" onclick="bbSeen()">.log</a>';
      h+='<a class="btn-main btn-blue" style="padding:4px 10px;font-size:.7em;flex:0" href="/blackbox/get?name='+nm+'&type=json" onclick="bbSeen()">.json</a>';
      h+='<button type="button" class="btn-main btn-stop" style="padding:4px 10px;font-size:.7em" onclick="bbDelete(\''+bbEsc(ev.name)+'\')">del</button>';
      h+='</div></div>';
    });
    el.innerHTML=h;
  }).catch(function(){});
}
function bbDelete(n){cmd('blackbox_delete',n);setTimeout(bbRefreshList,400);}
function bbDeleteAll(){if(confirm('Delete all recorded events from the device?')){cmd('blackbox_delete_all',true);bbSeen();setTimeout(bbRefreshList,400);}}
// ── Tap capability checker (#125) ──
var CAP_MSG={
  nag_killer:['0x370 + DAS state present — gates correctly',
              '0x370 here but no DAS state — dual-CAN recommended (read DAS on a second tap)',
              'no 0x370 on this tap — wrong bus for the nag killer'],
  ap_first:['DAS state readable',
            'dual-CAN',
            'no DAS state here (need 0x399/0x39B)'],
  fsd_activation:['AP control frame present (0x3FD/0x3EE)',
                  '',
                  'no AP control frame to modify here'],
  soft_engage:['0x129 steering angle present',
               '',
               'no 0x129 — degrades to AP-First-only'],
  body_control:['Vehicle/body bus reachable — mirror/window/lights frames present (RX only; not proof of actuation)',
                '',
                'not on this tap — needs the X179 A-pillar (Vehicle-bus) tap']};
function capRow(name,key,v){
  var col=v===0?'var(--accent)':v===1?'var(--yellow)':'var(--red)';
  var sym=v===0?'✓':v===1?'⚠':'✗';
  var msg=(CAP_MSG[key]||['','',''])[v]||'';
  return '<div style="display:flex;gap:8px;padding:5px 0;border-top:1px solid var(--border)">'
    +'<span style="color:'+col+';font-weight:700;flex:0 0 14px">'+sym+'</span>'
    +'<span style="flex:0 0 92px;font-size:.8em;color:var(--text)">'+name+'</span>'
    +'<span style="font-size:.73em;color:var(--text3);line-height:1.3">'+msg+'</span></div>';
}
function capSync(d){
  var c=d.capability;if(!c)return;
  pill('capSt',c.state===2,c.state===1?'Listening…':(c.state===2?'Done':'Idle'));
  var btn=document.getElementById('capBtn');if(btn)btn.disabled=(c.state===1);
  var el=document.getElementById('capBody');if(!el)return;
  if(c.state===0){el.innerHTML='<div style="font-size:.78em;color:var(--text3);padding:6px 0">Connect to run a check, or press Re-check.</div>';return;}
  if(c.state===1){el.innerHTML='<div style="font-size:.8em;color:var(--text2);padding:6px 0">Listening on the bus… '+Math.ceil((c.ms_left||0)/1000)+'s</div>';return;}
  var buses=c.buses||[];
  if(!buses.length){el.innerHTML='<div style="font-size:.8em;color:var(--yellow);padding:6px 0">No frames seen — check wiring / that the car is awake.</div>';return;}
  var dual=buses.length>1;
  var h='';
  buses.forEach(function(b){
    h+='<div style="margin-top:8px">';
    if(dual)h+='<div style="font-size:.78em;color:var(--text2);font-weight:600;margin-bottom:2px">'+bbEsc(b.bus)+' &middot; '+(b.frames||0)+' frames</div>';
    h+=capRow('Nag killer','nag_killer',b.nag_killer);
    h+=capRow('AP-First','ap_first',b.ap_first);
    h+=capRow('FSD activate','fsd_activation',b.fsd_activation);
    h+=capRow('Soft Engage','soft_engage',b.soft_engage);
    h+=capRow('Body/comfort bus','body_control',b.body_control);
    if(b.body_control===0){
      var seen=[];
      if(b.body_ui)seen.push('mirror/lock/horn');
      if(b.body_window)seen.push('windows');
      if(b.body_lights)seen.push('lights/turn');
      if(b.body_door)seen.push('mirror read-back');
      if(seen.length)h+='<div style="font-size:.7em;color:var(--text3);padding-top:3px">Reachable frames: '+seen.join(', ')+' (presence only — not proof injection actuates them).</div>';
    }
    var hint=b.hint?('Best guess: '+bbEsc(b.hint)+' — confirm in Service Mode → CAN Port'):'';
    if(hint)h+='<div style="font-size:.7em;color:var(--text3);padding-top:5px">'+hint+'</div>';
    if(b.hw_unconfirmed)h+='<div style="font-size:.7em;color:var(--yellow);padding-top:3px">HW unconfirmed — 0x399 reading assumed; verdict may change once HW is detected.</div>';
    h+='</div>';
  });
  el.innerHTML=h;
}
function ring(p){
  var b=document.getElementById('socBar');
  b.style.strokeDashoffset=CIRC-(CIRC*Math.min(p,100)/100);
  b.style.stroke=socCol(p);
}

function upd(d){
  if(!d || Date.now() < busy) return;
  // Status
  var apActive=!!d.ap_active;
  pill('fsdSt', apActive, apActive?'已激活':'等待');
  pill('opMode', d.op_mode===1, d.op_mode===1?'模拟计算（物理 TX 禁用）':'只监听（物理 TX 禁用）');

  var hwEl=document.getElementById('hwVer');
  if(hwEl){
    hwEl.className='pill '+(d.hw_version>0?'on':'off');
    hwEl.innerHTML='<span class="pd"></span>'+(HW[d.hw_version]||'?');
  }

  pill('nagSt', d.nag_killer, d.nag_killer?'ON':'OFF');
  pill('canVeh', d.can_vehicle_detected, d.can_vehicle_detected?'已检测':'无 CAN 数据');
  pill('bmsSt', d.bms && d.bms.seen, (d.bms && d.bms.seen)?'实时':'等待数据');
  var bF=document.getElementById('bmsFrames');
  if(bF) bF.textContent='HV:'+(d.bms_hv_seen||0)+' SOC:'+(d.bms_soc_seen||0)+' TH:'+(d.bms_thermal_seen||0);
  var bs=document.getElementById('batterySummary');
  if(bs){
    if(d.bms&&d.bms.seen){
      var soc=(d.bms.soc!==undefined)?Number(d.bms.soc).toFixed(0)+'%':'已检测';
      bs.textContent=soc;
      bs.style.color='var(--accent)';
    }else{
      bs.textContent='未检测';
      bs.style.color='var(--text3)';
    }
  }

  // OTA banner
  var otaB=document.getElementById('otaBanner');
  if(otaB){
    otaB.style.display=d.ota?'block':'none';
    if(d.ota) otaB.innerHTML=d.ignore_ota?'&#9888;&#xFE0F; OTA UPDATE IN PROGRESS &mdash; TX ALLOWED BY IGNORE OTA':'&#9888;&#xFE0F; OTA UPDATE IN PROGRESS &mdash; CAN TX SUSPENDED';
  }

  // Autopark banner (#180) — TX paused during an in-car Autopark episode
  var apB=document.getElementById('autoparkBanner');
  if(apB) apB.style.display=d.autopark_block?'block':'none';

  // Signal Map watchdog banner (#100)
  var smW=document.getElementById('sigmapWarn');
  if(smW) smW.style.display=d.signal_map_das_missing?'block':'none';

  // 14.x firmware warning banner
  var w14x=document.getElementById('warn14x');
  if(w14x) w14x.style.display=d.firmware_14x_warning?'block':'none';

  // Mode button
  var act=d.op_mode===1;
  var btn=document.getElementById('btnMode');
  if(btn){
    btn.textContent=act?'暂停模拟计算':'启用模拟计算';
    btn.className='btn-main '+(act?'btn-stop':'btn-act');
  }

  // Switches sync
  if(document.getElementById('swIgnoreOta')) document.getElementById('swIgnoreOta').checked=d.ignore_ota;
  // Manual HW selection (#110) — don't fight the user while the menu is open.
  var hwSel=document.getElementById('selHwOverride');
  if(hwSel && d.hw_override!==undefined && document.activeElement!==hwSel) hwSel.value=String(d.hw_override);

  var ps=document.getElementById('selProtocol');
  if(ps && d.fsd_protocol_mode!==undefined && document.activeElement!==ps) ps.value=String(d.fsd_protocol_mode);

  var ds=document.getElementById('selDriveStyle');
  if(ds && d.hw3_drive_style!==undefined && document.activeElement!==ds) ds.value=String(d.hw3_drive_style);
  var protoV13=(d.fsd_protocol_mode===13)||(d.fsd_protocol_mode===0&&d.hw_version===2);
  var protoV14=(d.fsd_protocol_mode===14)||(d.fsd_protocol_mode===0&&d.hw_version===3);
  var dsRow=document.getElementById('rowDriveStyle');
  if(dsRow) dsRow.style.display='flex';

  var sl=document.getElementById('spdLimitNow'); if(sl)sl.textContent=d.private399_limit_seen?Number(d.private399_limit_kph||0).toFixed(0):'--';
  var st=document.getElementById('spdOffTarget'); if(st)st.textContent=(d.hw3_offset_target!==undefined)?d.hw3_offset_target:'--';
  var sn=document.getElementById('spdOffNow'); if(sn)sn.textContent=(d.speed_offset!==undefined)?d.speed_offset:'--';
  var stn=document.getElementById('spdTargetNow'); if(stn)stn.textContent=d.private399_limit_seen?Number(d.research_target_kph||0).toFixed(0):'--';
  var tg=document.querySelectorAll('#targetGrid .target-cell'); if(tg&&tg.length){var _lim=d.private399_limit_seen?Math.round(Number(d.private399_limit_kph||0)):0;for(var _i=0;_i<tg.length;_i++)tg[_i].classList.toggle('active',Number(tg[_i].getAttribute('data-limit'))===_lim);}
  var d399=document.getElementById('diag399Limit'); if(d399)d399.textContent=d.private399_limit_seen?Number(d.private399_limit_kph||0).toFixed(0):'--';
  var d399r=document.getElementById('diag399Raw'); if(d399r)d399r.textContent=d.private399_limit_seen?String(d.private399_raw_limit):'--';
  var d3m=document.getElementById('diag3fdMux'); if(d3m)d3m.textContent=d.ap3fd_diag_seen?String(d.ap3fd_mux):'--';
  var d3o=document.getElementById('diag3fdOffset'); if(d3o)d3o.textContent=d.ap3fd_diag_seen?String(d.ap3fd_offset_raw):'--';
  var d3p=document.getElementById('diag3fdProfile'); if(d3p)d3p.textContent=d.ap3fd_diag_seen?String(d.ap3fd_profile_raw):'--';
  var d38=document.getElementById('diag3f8Follow'); if(d38)d38.textContent=(d.follow_distance_raw!==undefined)?String(d.follow_distance_raw):'--';
  speedSetVal('offsetMode',d.hw3_offset_mode);
  speedSetVal('manualOffset',d.hw3_manual_offset);
  if(Array.isArray(d.hw3_custom_pct))for(var i=0;i<4;i++)speedSetVal('cp'+i,d.hw3_custom_pct[i]);
  var rv=document.getElementById('researchValues');
  if(rv){var hwl=(d.hw_override&&d.hw_override!==0)?d.hw_override:d.hw_version;var hwt=hwl===3?'HW4':(hwl===2?'HW3':'未知');rv.textContent='目标 '+Number(d.research_target_kph||0).toFixed(2)+' / 平滑 '+Number(d.research_smooth_kph||0).toFixed(2)+' / cap '+Number(d.research_cap_kph||0).toFixed(0)+' km/h；布局 '+hwt+'；模拟帧 can0='+d.mock_can0+'，can1='+d.mock_can1+'；物理 TX=0';}

  // Compact 1.4.33-style overview. Mirrors existing telemetry only.
  var ql=document.getElementById('qLimit'); if(ql)ql.textContent=d.private399_limit_seen?Number(d.private399_limit_kph||0).toFixed(0):'--';
  var qt=document.getElementById('qTarget'); if(qt)qt.textContent=d.private399_limit_seen?Number(d.research_target_kph||0).toFixed(0):'--';
  var qo=document.getElementById('qOffset'); if(qo)qo.textContent=(d.speed_offset!==undefined)?String(d.speed_offset):'--';
  var qh=document.getElementById('qHw'); if(qh){var _hw=(d.hw_override&&d.hw_override!==0)?d.hw_override:d.hw_version;qh.textContent=_hw===3?'HW4':(_hw===2?'HW3':(_hw===1?'Legacy':'--'));}
  var qp=document.getElementById('qProto'); if(qp){var _p=d.fsd_protocol_mode;qp.textContent=_p===13?'V13':(_p===14?'V14':('Auto '+(d.research_protocol_detected?('V'+d.research_protocol_detected):'')));}
  var qtemp=document.getElementById('qTemp'); if(qtemp)qtemp.textContent=(d.chip_temp_c!==undefined)?Number(d.chip_temp_c).toFixed(1):'--';

  var ct=document.getElementById('chipTemp'),ctm=document.getElementById('chipTempMax');
  if(ct)ct.textContent=(d.chip_temp_c!==undefined)?Number(d.chip_temp_c).toFixed(1)+' °C':'--';
  if(ctm)ctm.textContent=(d.chip_temp_max_c!==undefined)?Number(d.chip_temp_max_c).toFixed(1)+' °C':'--';
  var tv=Number(d.chip_temp_c||0),tlabel='正常',twarn='';
  if(tv>=90){tlabel='过热';twarn='warn';}
  else if(tv>=80){tlabel='高温';twarn='warn';}
  else if(tv>=70){tlabel='偏高';}
  pill('chipTempSt',tv>0,tlabel,twarn);
  if(document.getElementById('swFsdUnlock')) document.getElementById('swFsdUnlock').checked=d.fsd_unlock;
  if(document.getElementById('swNag')) document.getElementById('swNag').checked=d.nag_killer;
  if(document.getElementById('swContinuousAp')) document.getElementById('swContinuousAp').checked=d.continuous_ap;
  if(document.getElementById('swApFirst')) document.getElementById('swApFirst').checked=d.ap_first;
  if(document.getElementById('swApFe')) document.getElementById('swApFe').checked=d.ap_first_edge;
  if(document.getElementById('swApMi')) document.getElementById('swApMi').checked=d.ap_first_minimal;
  if(document.getElementById('swNagF')) document.getElementById('swNagF').checked=d.nag_faithful;
  if(document.getElementById('swSoft')) document.getElementById('swSoft').checked=d.soft_engage;
  if(document.getElementById('swNagB')) document.getElementById('swNagB').checked=d.nag_burst;
  if(document.getElementById('swAbrt')) document.getElementById('swAbrt').checked=d.abort_guard;
  if(d.cfg_das_id!==undefined) setSig(d);
  if(document.getElementById('swBms')) document.getElementById('swBms').checked=d.bms_output;
  if(document.getElementById('swFsd')) document.getElementById('swFsd').checked=d.force_fsd;
  if(document.getElementById('swChina')) document.getElementById('swChina').checked=d.china_mode;
  if(document.getElementById('swChime')) document.getElementById('swChime').checked=d.suppress_speed_chime;
  if(document.getElementById('rowChime')) document.getElementById('rowChime').style.display=(d.isa_speed_enabled&&currentControlTier==='regular')?'flex':'none';
  if(document.getElementById('swTlssc')) document.getElementById('swTlssc').checked=d.tlssc_restore;
  if(document.getElementById('swSummon')) document.getElementById('swSummon').checked=d.summon_unlock;
  if(document.getElementById('swCog')) document.getElementById('swCog').checked=d.continue_on_green;
  if(document.getElementById('swTlssc38')) document.getElementById('swTlssc38').checked=d.assist_tlssc_bit38;
  if(document.getElementById('swRhd')) document.getElementById('swRhd').checked=d.assist_rhd_override;
  if(document.getElementById('swTelOff')) document.getElementById('swTelOff').checked=d.assist_telemetry_off;
  var apmv3Sel=document.getElementById('selApmv3');
  if(apmv3Sel && d.apmv3_branch!==undefined && document.activeElement!==apmv3Sel) apmv3Sel.value=String(d.apmv3_branch);
  if(document.getElementById('swTrkMode')) document.getElementById('swTrkMode').checked=d.track_mode_inject;
  if(document.getElementById('trkRot')&&(document.activeElement||{}).id!=='trkRot'&&d.track_rotation_pct!==undefined){document.getElementById('trkRot').value=d.track_rotation_pct;var _tr=document.getElementById('trkRotV');if(_tr)_tr.textContent=d.track_rotation_pct;}
  if(document.getElementById('trkStab')&&(document.activeElement||{}).id!=='trkStab'&&d.track_stability_pct!==undefined){document.getElementById('trkStab').value=d.track_stability_pct;var _ts=document.getElementById('trkStabV');if(_ts)_ts.textContent=d.track_stability_pct;}
  if(document.getElementById('swTrkPC')) document.getElementById('swTrkPC').checked=d.track_post_cooling;
  if(document.getElementById('swTrkCO')) document.getElementById('swTrkCO').checked=d.track_cmp_overclock;
  if(document.getElementById('swDisp')) document.getElementById('swDisp').checked=!!d.display_enabled;
  if((document.activeElement||{}).id!=='dispBr' && document.getElementById('dispBr'))
    document.getElementById('dispBr').value=d.display_brightness||50;
  if((document.activeElement||{}).id!=='dispTo' && document.getElementById('dispTo'))
    document.getElementById('dispTo').value=d.display_timeout_s||60;
  if(document.getElementById('swDump')) document.getElementById('swDump').checked=!!d.can_dump;

  if((document.activeElement||{}).id!=='numSleep' && document.getElementById('numSleep'))
    document.getElementById('numSleep').value=Math.floor((d.sleep_ms||0)/1000);

  updateControlsSummary(d);
  pill('dumpSt',d.can_dump,d.can_dump?'Recording':'Idle');

  // CAN stats
  if(document.getElementById('rxCnt')) document.getElementById('rxCnt').textContent=(d.rx_count||0).toLocaleString();
  if(document.getElementById('txCnt')) document.getElementById('txCnt').textContent=(d.tx_count||0).toLocaleString();
  if(document.getElementById('crcErr')) document.getElementById('crcErr').textContent=d.crc_errors||0;
  if(document.getElementById('crcSplit')) document.getElementById('crcSplit').textContent='RX\u00a0missed\u00a0'+(d.rx_missed_count||0)+' · bus\u00a0'+(d.bus_error_count||0)+' · TX\u00a0fail\u00a0'+(d.tx_failed_count||0);
  if(document.getElementById('fps')) document.getElementById('fps').textContent=(d.fps||0.0).toFixed(1);
  httpLogAllowed=true; // capture works in both modes — needed to log through an Activate (#108)
  if(!httpLogRunning)setHttpLogUi(false);
  if(!httpLogRunning){
    if(httpLogReady){
      pill('httpLogSt',true,'Ready');
    }else{
      pill('httpLogSt', d.http_can_stream && d.http_can_stream.active,
        httpLogAllowed?((d.http_can_stream && d.http_can_stream.active)?'Streaming':'Idle'):'Disabled');
    }
  }
  if(!httpLogRunning&&!httpLogReady&&document.getElementById('httpLogInfo')&&!httpLogAllowed)
    logInfo('HTTP CAN log is available only in Listen-Only mode.','var(--yellow)');
  if(document.getElementById('httpLogBuf'))
    document.getElementById('httpLogBuf').textContent=((d.http_can_stream&&d.http_can_stream.buffered)||0)+' frames';
  if(document.getElementById('httpLogDrop'))
    document.getElementById('httpLogDrop').textContent=((d.http_can_stream&&d.http_can_stream.dropped)||0)+' frames / rx-missed '+((d.http_can_stream&&d.http_can_stream.rx_missed)||0);
  if(document.getElementById('httpLogFiltered'))
    document.getElementById('httpLogFiltered').textContent=((d.http_can_stream&&d.http_can_stream.filtered)||0)+' frames';

  // Battery
  if(d.bms && d.bms.seen){
    var sn=document.getElementById('bSoc');
    if(sn){
      sn.textContent=d.bms.soc.toFixed(0)+'%';
      sn.style.color=socCol(d.bms.soc);
    }
    ring(d.bms.soc);
    if(document.getElementById('bVolt')) document.getElementById('bVolt').textContent=d.bms.voltage.toFixed(0)+'V';
    var ce=document.getElementById('bCurr');
    if(ce){
      ce.textContent=(d.bms.current>=0?'+':'')+d.bms.current.toFixed(1)+'A';
      ce.style.color=d.bms.current>=0?'var(--accent)':'var(--red)';
    }
    if(document.getElementById('bTemp')) document.getElementById('bTemp').textContent=d.bms.temp_min+'~'+d.bms.temp_max+'\u00b0C';
  }

  // Device
  if(document.getElementById('fwBuild')) document.getElementById('fwBuild').textContent=d.fw_build;
  if(document.getElementById('uptime')) document.getElementById('uptime').textContent=fmt(d.uptime_s||0);
  if(document.getElementById('wifiCl')) document.getElementById('wifiCl').textContent=d.wifi_clients||0;
  var partEl=document.getElementById('otaPartInfo');
  if(partEl && d.ota_partition){
    var p=d.ota_partition;
    var stateStr=(p.state===0)?'New':(p.state===1)?'Pending':(p.state===2)?'Valid':(p.state===3)?'Invalid':'State '+p.state;
    partEl.textContent=p.running+' ('+stateStr+') - '+(p.has_ota?'OTA capable':'No OTA partition');
    var info=document.getElementById('otaRollbackInfo');
    if(info && !p.has_ota){
      info.innerHTML='<b style="color:var(--red)">No OTA Partition</b><br>This build appears to be running from a factory/single app partition. Use an OTA partition table before relying on Web updates.';
    }
  }
}

function uploadFirmware(){
  var input=document.getElementById('otaFile');
  var file=input.files[0];
  if(!file)return;
  if(!file.name.endsWith('.bin')){alert('Error: Please select a .bin firmware file');input.value='';return;}
  var MAX_SIZE=16*1024*1024;
  if(file.size>MAX_SIZE){alert('Error: Firmware file too large (max 16MB)');input.value='';return;}
  if(file.size<32768 && !confirm('Warning: This file is very small ('+Math.round(file.size/1024)+' KB).\nAre you sure it is a valid ESP32 firmware?')){input.value='';return;}
  if(!confirm('Flash firmware: '+file.name+' ('+Math.round(file.size/1024)+' KB)?\n\nDevice will reboot after update.')){input.value='';return;}
  var prog=document.getElementById('otaProgress'),bar=document.getElementById('otaBar'),status=document.getElementById('otaStatus'),bytes=document.getElementById('otaBytes'),btn=document.getElementById('otaSelectBtn');
  prog.style.display='block';bar.style.width='0%';bar.style.background='var(--accent)';status.textContent='Uploading firmware...';status.style.color='var(--text2)';bytes.textContent='0 / '+Math.round(file.size/1024)+' KB';btn.disabled=true;btn.style.opacity='.5';
  var xhr=new XMLHttpRequest();
  xhr.upload.addEventListener('progress',function(e){if(e.lengthComputable){var pct=Math.round((e.loaded/e.total)*100);bar.style.width=pct+'%';status.textContent='Uploading: '+pct+'%';bytes.textContent=Math.round(e.loaded/1024)+' / '+Math.round(e.total/1024)+' KB';}});
  xhr.addEventListener('load',function(){btn.disabled=false;btn.style.opacity='1';if(xhr.status===200&&xhr.responseText==='OK'){bar.style.width='100%';status.textContent='Upload complete - rebooting...';status.style.color='var(--accent)';var c=8;var t=setInterval(function(){c--;bytes.textContent='Reconnecting in '+c+'s...';if(c<=0){clearInterval(t);location.reload();}},1000);}else{bar.style.background='var(--red)';status.textContent='Update failed';status.style.color='var(--red)';bytes.textContent='Server response: '+(xhr.responseText||xhr.statusText||'Unknown error');input.value='';}});
  xhr.addEventListener('error',function(){btn.disabled=false;btn.style.opacity='1';bar.style.background='var(--red)';status.textContent='Connection lost during upload';status.style.color='var(--red)';bytes.textContent='Check WiFi connection and try again';input.value='';});
  xhr.addEventListener('timeout',function(){btn.disabled=false;btn.style.opacity='1';bar.style.background='var(--yellow)';status.textContent='Upload timed out';status.style.color='var(--yellow)';bytes.textContent='The device may have rebooted - check if new firmware is running';input.value='';});
  var fd=new FormData();fd.append('firmware',file);xhr.open('POST','/update',true);xhr.setRequestHeader('Authorization',authHeader);xhr.timeout=120000;xhr.send(fd);
}

function selectFirmware(el){
  requireAuth(function(){document.getElementById('otaFile').click();},el);
}

function restartDevice(el){
  restartAnchor=el;
  requireAuth(function(){showRestartConfirm(el);},el);
}

function movePanelNear(panel,anchor){
  if(!panel || !anchor)return;
  var card=anchor.closest?anchor.closest('.card'):null;
  if(card && panel.parentNode!==card)card.appendChild(panel);
}

function showRestartConfirm(anchor){
  var p=document.getElementById('restartConfirmPanel');
  movePanelNear(p,anchor);
  if(p)p.className='confirm-panel show';
}

function cancelRestartConfirm(){
  var p=document.getElementById('restartConfirmPanel');
  if(p)p.className='confirm-panel';
}

function confirmRestart(){
  cancelRestartConfirm();
  requestRestart();
}

function requestRestart(){
  fetch('/restart',{headers:{Authorization:authHeader}}).then(function(r){
    if(!r.ok){authHeader='';requireAuth(function(){showRestartConfirm(restartAnchor);},restartAnchor);return;}
    alert('Device restart triggered');
    setTimeout(function(){location.reload();},8000);
  }).catch(function(){setTimeout(function(){location.reload();},8000);});
}

function requireAuth(action,anchor){
  if(authHeader && checkAuth()){action();return;}
  authHeader='';
  authAction=action;
  showAuth(anchor);
}

function showAuth(anchor){
  var m=document.getElementById('authPanel');
  var u=document.getElementById('authUser');
  var p=document.getElementById('authPass');
  movePanelNear(m,anchor);
  if(u)u.value='admin';
  if(p)p.value='';
  if(m)m.className='auth-panel show';
  setTimeout(function(){if(u)u.focus();},0);
}

function cancelAuth(){
  authAction=null;
  var m=document.getElementById('authPanel');
  if(m)m.className='auth-panel';
}

function submitAuth(){
  var u=document.getElementById('authUser');
  var p=document.getElementById('authPass');
  var user=u?u.value:'';
  var pass=p?p.value:'';
  if(!user || !pass)return;
  authHeader='Basic '+btoa(user+':'+pass);
  if(!checkAuth()){
    authHeader='';
    authFailed();
    if(p){p.value='';p.focus();}
    return;
  }
  var action=authAction;
  cancelAuth();
  if(action)action();
}

function checkAuth(){
  try{
    var xhr=new XMLHttpRequest();
    xhr.open('GET','/auth',false);
    xhr.setRequestHeader('Authorization',authHeader);
    xhr.send(null);
    return xhr.status===200;
  }catch(e){
    return false;
  }
}

function authFailed(){
  alert('Authentication failed');
  var input=document.getElementById('otaFile');
  if(input)input.value='';
}

function sdFormat(){
  if(!confirm('Format SD card? All data will be lost.'))return;
  var btn=document.getElementById('btnFmt');
  var out=document.getElementById('fmtOut');
  btn.disabled=true;btn.textContent='FORMATTING\u2026';
  fetch('/sdformat').then(function(r){return r.json();}).then(function(d){
    out.style.display='block';
    out.style.color=d.ok?'var(--accent)':'var(--red)';
    out.textContent=d.msg+(d.ok?' \u2014 '+d.free_mb+' MB free':'');
  }).catch(function(){
    out.style.display='block';out.style.color='var(--red)';out.textContent='Request failed';
  }).then(function(){
    btn.disabled=false;btn.textContent='FORMAT SD CARD';
  });
}
function saveWifi(){
  var s=document.getElementById('wifiSsid').value;
  var p=document.getElementById('wifiPass').value;
  var h=document.getElementById('swWifiHid').checked;
  var ss=document.getElementById('wifiStaSsid').value;
  var sp=document.getElementById('wifiStaPass').value;
  if(s.length<1){alert('SSID required');return;}
  if(p!=='***' && p.length>0 && p.length<8){alert('Password must be empty or 8+ chars');return;}
  if(sp!=='***' && ss.length>0 && sp.length>0 && sp.length<8){alert('Network password must be empty or 8+ chars');return;}
  if(confirm('WiFi settings will be updated and the device will restart.')){
    var b=document.activeElement; if(b&&b.tagName==='BUTTON'){b.disabled=true;b.textContent='SAVING...';}
    cmd('wifi_cfg',{ssid:s,pass:p,hidden:h,sta_ssid:ss,sta_pass:sp});
  }
}
function speedNum(cmdName,el,minv,maxv){
  var v=parseInt(el.value,10); if(isNaN(v))v=minv; v=Math.max(minv,Math.min(maxv,v)); el.value=v; cmd(cmdName,v);
}
function speedSetVal(id,value){
  var el=document.getElementById(id);
  if(el && document.activeElement!==el && value!==undefined)el.value=String(value);
}
function cmd(c,v){
  if(ws&&ws.readyState===1) {
    ws.send(JSON.stringify({cmd:c,value:v}));
    busy = Date.now() + 3000;
  }
}
function toggleMode(){ cmd('mode',null); }
function gv(id){ var e=document.getElementById(id); return (e&&e.value.trim()!=='')?e.value.trim():'0'; }
function saveSigCfg(){
  var csv=[gv('cgDid'),gv('cgApB'),gv('cgApS'),gv('cgApM'),
           gv('cgHoB'),gv('cgHoS'),gv('cgHoM'),
           gv('cgSid'),gv('cgSHi'),gv('cgSLo')].join(',');
  cmd('sig_cfg',csv);
}
function sv(id,val){ var e=document.getElementById(id); if(e&&e!==document.activeElement) e.value=val; }
function hx(n){ return n?('0x'+n.toString(16).toUpperCase()):'0'; }
// ── Built-in variant-profile auto-suggest (#126) ──
var PM_SUG=null;
function pmSync(d){
  var p=d.profile;var el=document.getElementById('pmSuggest');if(!el)return;
  if(p&&p.suggest){
    PM_SUG=p;
    var nm=document.getElementById('pmName');if(nm)nm.textContent=p.name||'?';
    el.style.display='block';
  } else {
    PM_SUG=null;el.style.display='none';
  }
}
function sf(id,val){ var e=document.getElementById(id); if(e) e.value=val; }
function pmApply(){
  var p=PM_SUG;if(!p)return;
  // One-tap confirm: fill the Signal Map fields from the suggested profile and
  // apply via the existing sig_cfg path. Steer mapping is left as-is (0 unless
  // the user already set it). Never applied without this tap.
  sf('cgDid',hx(p.das_id));
  sf('cgApB',p.apb); sf('cgApS',p.aps); sf('cgApM',hx(p.apm));
  sf('cgHoB',p.hob); sf('cgHoS',p.hos); sf('cgHoM',hx(p.hom));
  saveSigCfg();
}
function setSig(d){
  sv('cgDid',hx(d.cfg_das_id)); sv('cgApB',d.cfg_apb); sv('cgApS',d.cfg_aps); sv('cgApM',hx(d.cfg_apm));
  sv('cgHoB',d.cfg_hob); sv('cgHoS',d.cfg_hos); sv('cgHoM',hx(d.cfg_hom));
  sv('cgSid',hx(d.cfg_steer_id)); sv('cgSHi',d.cfg_shi); sv('cgSLo',d.cfg_slo);
}

function logInfo(text,color){
  var e=document.getElementById('httpLogInfo');
  if(!e)return;
  e.textContent=text;
  e.style.color=color||'var(--text2)';
}

function setHttpLogUi(running){
  var btn=document.getElementById('btnHttpLog');
  if(btn){
    btn.disabled=!httpLogAllowed&&!httpLogReady;
    btn.style.opacity=btn.disabled?'.45':'1';
    if(running){
      btn.textContent='STOP COLLECTING';
      btn.className='btn-main btn-stop';
    }else if(httpLogReady){
      btn.textContent='SAVE LOG FILE';
      btn.className='btn-main btn-blue';
    }else if(!httpLogAllowed){
      btn.textContent='LISTEN-ONLY REQUIRED';
      btn.className='btn-main btn-blue';
    }else{
      btn.textContent='STREAM LOG AND SAVE';
      btn.className='btn-main btn-blue';
    }
  }
  var filterEl=document.getElementById('httpLogFilter');
  if(filterEl)filterEl.disabled=running;
  pill('httpLogSt',running||httpLogReady,running?'Collecting':(httpLogReady?'Ready':'Idle'));
}

function formatBytes(n){
  if(n<1024)return n+' B';
  if(n<1048576)return (n/1024).toFixed(1)+' KB';
  return (n/1048576).toFixed(1)+' MB';
}

function logFileName(){
  var d=new Date();
  function p(n){return n<10?'0'+n:''+n;}
  return 'tesla_can_'+d.getFullYear()+p(d.getMonth()+1)+p(d.getDate())+'_'
    +p(d.getHours())+p(d.getMinutes())+p(d.getSeconds())+'.dump';
}

function clearHttpLogBlob(){
  if(httpLogSaveUrl)URL.revokeObjectURL(httpLogSaveUrl);
  httpLogSaveUrl='';
  httpLogName='';
  httpLogReady=false;
}

function prepareHttpLogFile(){
  clearHttpLogBlob();
  if(httpLogBytes===0){
    logInfo('No CAN frames collected. Nothing saved.','var(--yellow)');
    httpLogParts=[];
    setHttpLogUi(false);
    return;
  }
  httpLogName=logFileName();
  httpLogReady=true;
  setHttpLogUi(false);
  logInfo('Log ready in phone memory: '+httpLogName+' ('+formatBytes(httpLogBytes)+'). Tap SAVE LOG FILE.','var(--accent)');
}

function savePreparedHttpLog(){
  if(!httpLogReady||httpLogBytes===0)return;
  var blob=new Blob(httpLogParts,{type:'text/plain;charset=utf-8'});
  if(blob.size===0){
    logInfo('No CAN frames collected. Nothing saved.','var(--yellow)');
    return;
  }
  if(window.File&&navigator.canShare&&navigator.share){
    var file=new File([blob],httpLogName,{type:'text/plain'});
    if(navigator.canShare({files:[file]})){
      navigator.share({files:[file],title:httpLogName}).then(function(){
        logInfo('Save/share requested for '+httpLogName+'.','var(--accent)');
      }).catch(function(err){
        logInfo('Share cancelled or failed: '+(err&&err.message?err.message:'unknown')+'. Trying download link...','var(--yellow)');
        downloadHttpLogBlob(blob);
      });
      return;
    }
  }
  downloadHttpLogBlob(blob);
}

function downloadHttpLogBlob(blob){
  if(httpLogSaveUrl)URL.revokeObjectURL(httpLogSaveUrl);
  httpLogSaveUrl=URL.createObjectURL(blob);
  var a=document.createElement('a');
  a.href=httpLogSaveUrl;
  a.download=httpLogName;
  a.rel='noopener';
  a.textContent=httpLogName;
  a.style.position='fixed';
  a.style.left='0';
  a.style.bottom='0';
  a.style.opacity='0.01';
  document.body.appendChild(a);
  a.click();
  setTimeout(function(){a.remove();},1000);
  logInfo('Save requested for '+httpLogName+'. If no file appears, tap SAVE LOG FILE again or use another browser.','var(--accent)');
}

function stopHttpLog(reason){
  if(!httpLogRunning)return;
  httpLogRunning=false;
  if(httpLogReader)httpLogReader.cancel().catch(function(){});
  if(httpLogAbort)httpLogAbort.abort();
  httpLogReader=null;
  httpLogAbort=null;
  prepareHttpLogFile();
  if(reason){
    if(httpLogReady){
      logInfo(reason+' Log ready in phone memory: '+httpLogName+' ('+formatBytes(httpLogBytes)+'). Tap SAVE LOG FILE.','var(--yellow)');
    }else{
      logInfo(reason,'var(--yellow)');
    }
  }
}

function startHttpLog(){
  if(httpLogRunning)return;
  if(!httpLogAllowed){
    logInfo('Switch to Listen-Only mode before starting HTTP CAN log.','var(--yellow)');
    setHttpLogUi(false);
    return;
  }
  if(!window.ReadableStream){
    alert('This browser does not support HTTP stream collection.');
    return;
  }
  httpLogParts=[];
  httpLogBytes=0;
  clearHttpLogBlob();
  httpLogStarted=Date.now();
  httpLogRunning=true;
  httpLogAbort=new AbortController();
  setHttpLogUi(true);
  logInfo('Connecting to HTTP stream...');

  var streamUrl='http://'+location.hostname+':82/stream';
  var filterEl=document.getElementById('httpLogFilter');
  var filter=(filterEl&&filterEl.value)?filterEl.value.trim():'';
  if(filter)streamUrl+='?ids='+encodeURIComponent(filter);

  fetch(streamUrl,{cache:'no-store',signal:httpLogAbort.signal})
    .then(function(r){
      if(!r.ok)throw new Error('HTTP '+r.status);
      if(!r.body)throw new Error('Readable stream unavailable');
      logInfo('Collecting 0 B...');
      var reader=r.body.getReader();
      httpLogReader=reader;
      function pump(){
        return reader.read().then(function(result){
          if(result.done)return;
          if(result.value&&result.value.length){
            httpLogParts.push(result.value);
            httpLogBytes+=result.value.length;
            var secs=Math.max(1,Math.round((Date.now()-httpLogStarted)/1000));
            logInfo('Collecting '+formatBytes(httpLogBytes)+' for '+secs+'s...');
          }
          return pump();
        });
      }
      return pump();
    })
    .catch(function(err){
      if(!httpLogRunning)return;
      httpLogRunning=false;
      httpLogReader=null;
      httpLogAbort=null;
      setHttpLogUi(false);
      logInfo('Stream stopped: '+(err&&err.message?err.message:'connection closed'),'var(--yellow)');
      if(httpLogBytes>0)prepareHttpLogFile();
    });
}

function toggleHttpLog(){
  if(httpLogRunning)stopHttpLog();
  else if(httpLogReady)savePreparedHttpLog();
  else startHttpLog();
}

// ── Aux status poll (#124) ──
// blackbox/capability/profile are served on demand from /api/aux, NOT in the
// hot 1 Hz WS state push. Poll them on a slower timer so the black-box card,
// Tap Check card and profile suggestion stay live without bloating each push.
var auxBusy=false;
function auxPoll(){
  if(auxBusy)return;
  auxBusy=true;
  fetch('/api/aux').then(function(r){return r.json()}).then(function(a){
    if(a){bbSync(a);capSync(a);pmSync(a);}
  }).catch(function(){}).then(function(){auxBusy=false;});
}
setInterval(auxPoll,2500);

function conn(){
  ws=new WebSocket('ws://'+location.hostname+':81/');
  ws.onopen=function(){
    document.getElementById('dot').className='cdot';
    document.getElementById('connErr').style.display='none';
    clearTimeout(rt);
    auxPoll();
  };
  ws.onmessage=function(e){ try{var d=JSON.parse(e.data);initWifi(d);upd(d);}catch(x){} };
  ws.onclose=function(){
    document.getElementById('dot').className='cdot off';
    document.getElementById('connErr').style.display='block';
    rt=setTimeout(conn,2000);
  };
  ws.onerror=function(){ ws.close(); };
}
conn();
</script>
</body>
</html>
)rawliteral";

// ── JSON helpers ──────────────────────────────────────────────────────────────
static String json_escape(const char *s) {
    String out;
    for (; *s; ++s) {
        if (*s == '"')       out += "\\\"";
        else if (*s == '\\') out += "\\\\";
        else                 out += *s;
    }
    return out;
}

// ── JSON builder ──────────────────────────────────────────────────────────────
static String build_json() {
    // build_json() runs from the Arduino loopTask, including from the
    // WebSocket CONNECT callback. FSDState has grown large enough that keeping
    // a full snapshot as a local variable can exhaust the loopTask stack on a
    // browser refresh/reconnect (stack-canary panic in snprintf/build_json).
    // Keep the snapshot in static storage instead; state_copy() still refreshes
    // the complete snapshot before every JSON build.
    static FSDState state;
    if (!state_copy(&state)) return "{}";

    uint32_t uptime_s = (millis() - g_start_ms) / 1000;
    bool can_vehicle_detected = false;
    if (state.rx_count > 0) {
        can_vehicle_detected = (millis() - g_last_can_seen_ms) <= CAN_VEHICLE_ALIVE_MS;
    }

    // BMS sub-object
    char bms[128];
    if (state.bms_seen) {
        snprintf(bms, sizeof(bms),
            "{\"seen\":true,\"voltage\":%.1f,\"current\":%.1f,"
            "\"soc\":%.1f,\"temp_min\":%d,\"temp_max\":%d}",
            state.pack_voltage_v,
            state.pack_current_a,
            state.soc_percent,
            (int)state.batt_temp_min_c,
            (int)state.batt_temp_max_c);
    } else {
        strcpy(bms, "{\"seen\":false}");
    }

    char ota_part[128] = {};
    {
        const esp_partition_t *running = esp_ota_get_running_partition();
        const char *running_label = running ? running->label : "unknown";
        esp_ota_img_states_t ota_state = ESP_OTA_IMG_UNDEFINED;
        if (running) esp_ota_get_state_partition(running, &ota_state);
        bool has_ota = (running &&
            (running->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_0 ||
             running->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_1));
        snprintf(ota_part, sizeof(ota_part),
            "{\"running\":\"%s\",\"state\":%d,\"has_ota\":%s}",
            running_label, (int)ota_state, has_ota ? "true" : "false");
    }

    // fps as fixed-point string
    char fps_s[12];
    snprintf(fps_s, sizeof(fps_s), "%.1f", g_fps);

    // Combined CAN error count (legacy crc_errors key) and its per-cause split,
    // taken from one read so the dashboard tile always equals its breakdown.
    // On a busy bus it is mostly rx_missed (controller RX-queue drops).
    CanErrorSplit err = can_error_split(g_can_buses, g_can_count);
    uint32_t err_total = err.rx_missed_count + err.bus_error_count + err.tx_failed_count;

    String j;
    bool isa_speed_enabled = state.hw_version == TeslaHW_HW4;
    const char *ap_das_profile =
        (state.hw_version == TeslaHW_HW4) ? "HW4: DAS 0x39B + ISA 0x399" :
        (state.hw_version == TeslaHW_HW3) ? "HW3: DAS 0x399" :
        (state.hw_version == TeslaHW_Legacy) ? "Legacy: DAS 0x399" :
        "Waiting for HW detection";
    // Core payload runs ~1.3-1.5 KB of scalar fields plus small bms/ota/
    // http_can_stream objects — back near the beta.11 shape now that the heavy
    // blackbox/capability/profile blocks fetch from /api/aux (#124). 1.5 KB
    // avoids per-call String reallocs; build_json() runs on every WS push.
    j.reserve(2048);
    j  = "{";
    j += "\"fsd_enabled\":";   j += state.fsd_enabled                 ? "true" : "false"; j += ',';
    j += "\"ap_active\":";     j += state.ap_active                   ? "true" : "false"; j += ',';
    j += "\"op_mode\":";       j += (int)state.op_mode;                j += ',';
    j += "\"hw_override\":";   j += (int)state.hw_override;            j += ',';
    j += "\"hw_version\":";    j += (int)state.hw_version;             j += ',';
    j += "\"fsd_protocol_mode\":"; j += (int)state.fsd_protocol_mode;   j += ',';
    j += "\"hw3_drive_style\":"; j += (int)state.hw3_drive_style;       j += ',';
    j += "\"speed_profile_locked\":"; j += state.hw3_drive_style != 0 ? "true" : "false"; j += ',';
    j += "\"speed_profile\":"; j += state.research_profile; j += ',';
    j += "\"speed_offset\":"; j += state.speed_offset; j += ',';
    j += "\"speed_limit_seen\":"; j += state.speed_limit_seen ? "true" : "false"; j += ',';
    j += "\"speed_limit_kph\":"; j += String(state.speed_limit_kph, 1); j += ',';
    j += "\"private399_limit_seen\":"; j += state.private399_limit_seen ? "true" : "false"; j += ',';
    j += "\"private399_raw_limit\":"; j += (int)state.private399_raw_limit; j += ',';
    j += "\"private399_limit_kph\":"; j += String(state.private399_limit_kph, 1); j += ',';
    j += "\"ap3fd_diag_seen\":"; j += state.ap3fd_diag_seen ? "true" : "false"; j += ',';
    j += "\"ap3fd_mux\":"; j += (int)state.ap3fd_mux; j += ',';
    j += "\"ap3fd_offset_raw\":"; j += (int)state.ap3fd_offset_raw; j += ',';
    j += "\"ap3fd_profile_raw\":"; j += (int)state.ap3fd_profile_raw; j += ',';
    j += "\"follow_distance_raw\":"; j += (int)state.follow_distance_raw; j += ',';
    j += "\"physical_can_tx\":false,";
    j += "\"mock_can0\":"; j += state.research_mock_count[0]; j += ',';
    j += "\"mock_can1\":"; j += state.research_mock_count[1]; j += ',';
    j += "\"research_target_kph\":"; j += state.research_target_kph; j += ',';
    j += "\"research_smooth_kph\":"; j += state.hw3_smooth_target_kph; j += ',';
    j += "\"research_cap_kph\":"; j += state.research_cap_kph; j += ',';
    j += "\"research_protocol_detected\":"; j += state.research_protocol_detected; j += ',';
    j += "\"research_profile\":"; j += state.research_profile; j += ',';
    j += "\"hw3_offset_mode\":"; j += (int)state.hw3_offset_mode; j += ',';
    j += "\"hw3_manual_offset\":"; j += (int)state.hw3_manual_offset; j += ',';
    j += "\"hw3_custom_pct\":[";
    for (int i=0;i<4;i++){ if(i)j+=','; j+=(int)state.hw3_custom_pct[i]; } j += "],";
    j += "\"hw3_auto_speed\":"; j += state.hw3_auto_speed ? "true" : "false"; j += ',';
    j += "\"hw3_custom_speed\":"; j += state.hw3_custom_speed ? "true" : "false"; j += ',';
    j += "\"hw3_custom_target\":[";
    for (int i=0;i<5;i++){ if(i)j+=','; j+=(int)state.hw3_custom_target[i]; } j += "],";
    j += "\"hw3_offset_slew\":"; j += state.hw3_offset_slew ? "true" : "false"; j += ',';
    j += "\"hw3_slew_rate\":"; j += (int)state.hw3_slew_rate; j += ',';
    j += "\"hw3_offset_target\":"; j += (int)state.hw3_offset_target; j += ',';
    j += "\"hw3_high_speed_enable\":"; j += state.hw3_high_speed_enable ? "true" : "false"; j += ',';
    j += "\"hw3_high_speed_pct\":[";
    for (int i=0;i<5;i++){ if(i)j+=','; j+=(int)state.hw3_high_speed_pct[i]; } j += "],";
    j += "\"hw4_offset\":"; j += (int)state.hw4_offset; j += ',';
    j += "\"chip_temp_c\":"; j += String(g_chip_temp_c, 1); j += ',';
    j += "\"chip_temp_max_c\":"; j += String(g_chip_temp_max_c, 1); j += ',';
    j += "\"ota\":";           j += state.tesla_ota_in_progress        ? "true" : "false"; j += ',';
    j += "\"autopark_block\":"; j += state.autopark_tx_block            ? "true" : "false"; j += ',';
    j += "\"signal_map_das_missing\":"; j += state.signal_map_das_missing ? "true" : "false"; j += ',';
    j += "\"ap_das_profile\":\""; j += ap_das_profile;                 j += "\",";
    j += "\"isa_speed_enabled\":"; j += isa_speed_enabled              ? "true" : "false"; j += ',';
    j += "\"ignore_ota\":";    j += state.ignore_ota                   ? "true" : "false"; j += ',';
    j += "\"fsd_unlock\":";    j += state.fsd_unlock                   ? "true" : "false"; j += ',';
    j += "\"nag_killer\":";    j += state.nag_killer                   ? "true" : "false"; j += ',';
    j += "\"continuous_ap\":"; j += state.continuous_ap                 ? "true" : "false"; j += ',';
    j += "\"ap_first\":";      j += state.ap_first                      ? "true" : "false"; j += ',';
    j += "\"ap_first_edge\":"; j += state.ap_first_edge                 ? "true" : "false"; j += ',';
    j += "\"ap_first_minimal\":"; j += state.ap_first_minimal           ? "true" : "false"; j += ',';
    j += "\"nag_faithful\":";  j += state.nag_epas_faithful             ? "true" : "false"; j += ',';
    j += "\"soft_engage\":";   j += state.soft_engage                  ? "true" : "false"; j += ',';
    j += "\"nag_burst\":";     j += state.nag_burst                    ? "true" : "false"; j += ',';
    j += "\"abort_guard\":";   j += state.abort_guard                  ? "true" : "false"; j += ',';
    j += "\"cfg_das_id\":";    j += state.cfg_das_id;       j += ',';
    j += "\"cfg_apb\":";       j += state.cfg_apstate_byte;  j += ',';
    j += "\"cfg_aps\":";       j += state.cfg_apstate_shift; j += ',';
    j += "\"cfg_apm\":";       j += state.cfg_apstate_mask;  j += ',';
    j += "\"cfg_hob\":";       j += state.cfg_handson_byte;  j += ',';
    j += "\"cfg_hos\":";       j += state.cfg_handson_shift; j += ',';
    j += "\"cfg_hom\":";       j += state.cfg_handson_mask;  j += ',';
    j += "\"cfg_steer_id\":";  j += state.cfg_steer_id;      j += ',';
    j += "\"cfg_shi\":";       j += state.cfg_steer_hi;      j += ',';
    j += "\"cfg_slo\":";       j += state.cfg_steer_lo;      j += ',';
    j += "\"bms_output\":";    j += state.bms_output                   ? "true" : "false"; j += ',';
    j += "\"force_fsd\":";     j += state.force_fsd                    ? "true" : "false"; j += ',';
    j += "\"china_mode\":";    j += state.china_mode                   ? "true" : "false"; j += ',';
    j += "\"suppress_speed_chime\":"; j += state.suppress_speed_chime  ? "true" : "false"; j += ',';
    j += "\"tlssc_restore\":"; j += state.tlssc_restore                ? "true" : "false"; j += ',';
    j += "\"summon_unlock\":"; j += state.summon_unlock                ? "true" : "false"; j += ',';
    j += "\"continue_on_green\":"; j += state.continue_on_green         ? "true" : "false"; j += ',';
    j += "\"assist_tlssc_bit38\":"; j += state.assist_tlssc_bit38       ? "true" : "false"; j += ',';
    j += "\"assist_rhd_override\":"; j += state.assist_rhd_override      ? "true" : "false"; j += ',';
    j += "\"assist_telemetry_off\":"; j += state.assist_telemetry_off    ? "true" : "false"; j += ',';
    j += "\"apmv3_branch\":";  j += (int)state.apmv3_branch;             j += ',';
    j += "\"track_mode_inject\":"; j += state.track_mode_inject         ? "true" : "false"; j += ',';
    j += "\"track_rotation_pct\":";  j += (int)state.track_rotation_pct;   j += ',';
    j += "\"track_stability_pct\":"; j += (int)state.track_stability_pct;  j += ',';
    j += "\"track_post_cooling\":"; j += state.track_post_cooling        ? "true" : "false"; j += ',';
    j += "\"track_cmp_overclock\":"; j += state.track_cmp_overclock       ? "true" : "false"; j += ',';
    j += "\"firmware_14x_warning\":"; j += state.firmware_14x_warning  ? "true" : "false"; j += ',';
#if defined(BOARD_TTGO_DISPLAY)
    j += "\"display_enabled\":"; j += state.display_enabled             ? "true" : "false"; j += ',';
    j += "\"display_brightness\":"; j += state.display_brightness;      j += ',';
    j += "\"display_timeout_s\":";  j += state.display_timeout_s;       j += ',';
#endif
    j += "\"can_vehicle_detected\":"; j += can_vehicle_detected       ? "true" : "false"; j += ',';
    j += "\"bms_hv_seen\":";   j += state.seen_bms_hv;                 j += ',';
    j += "\"bms_soc_seen\":";  j += state.seen_bms_soc;                j += ',';
    j += "\"bms_thermal_seen\":"; j += state.seen_bms_thermal;          j += ',';
    j += "\"rx_count\":";      j += state.rx_count;                    j += ',';
    j += "\"tx_count\":";      j += state.tx_count;                    j += ',';
    j += "\"tx_modified\":";   j += state.frames_modified;             j += ',';
    j += "\"crc_errors\":";    j += err_total;                         j += ',';
    j += "\"rx_missed_count\":"; j += err.rx_missed_count;             j += ',';
    j += "\"bus_error_count\":"; j += err.bus_error_count;             j += ',';
    j += "\"tx_failed_count\":"; j += err.tx_failed_count;             j += ',';
    j += "\"fps\":";           j += fps_s;                             j += ',';
    j += "\"bms\":";           j += bms;                               j += ',';
    j += "\"uptime_s\":";      j += uptime_s;                          j += ',';
    j += "\"fw_build\":\"";    j += __DATE__;  j += ' '; j += __TIME__; j += "\",";
    j += "\"can_dump\":";      j += can_dump_active()                 ? "true" : "false"; j += ',';
    // blackbox/capability/profile moved OFF the hot WS state push (#124): they
    // are large and blackbox_status_json() scans the LittleFS dir every call.
    // The dashboard polls them on a slower timer via GET /api/aux instead.
    j += "\"sleep_ms\":";     j += state.sleep_idle_ms;               j += ',';
    j += "\"wifi_ssid\":\"";  j += json_escape(state.wifi_ssid);      j += "\",";
    j += "\"wifi_pass\":\"";  j += state.wifi_pass[0] ? "***" : "";  j += "\",";
    j += "\"wifi_hidden\":";  j += state.wifi_hidden                  ? "true" : "false"; j += ',';
    j += "\"wifi_sta_ssid\":\""; j += json_escape(state.wifi_sta_ssid); j += "\",";
    j += "\"wifi_sta_pass\":\""; j += state.wifi_sta_pass[0] ? "***" : ""; j += "\",";
    j += "\"wifi_clients\":";  j += (int)WiFi.softAPgetStationNum();   j += ',';
    j += "\"http_can_stream\":{";
    j += "\"active\":";       j += http_can_stream_active()           ? "true" : "false"; j += ',';
    j += "\"sent\":";         j += http_can_stream_frames_sent();      j += ',';
    j += "\"dropped\":";      j += http_can_stream_frames_dropped();   j += ',';
    j += "\"rx_missed\":";    j += http_can_stream_rx_missed();        j += ',';
    j += "\"filtered\":";     j += http_can_stream_frames_filtered();  j += ',';
    j += "\"buffered\":";     j += http_can_stream_buffered_frames();  j += "},";
    j += "\"ota_partition\":"; j += ota_part;
    j += '}';
    return j;
}

// Heavy/auxiliary status blocks, served on demand via GET /api/aux and polled
// by the dashboard on a slow timer — kept OUT of the 1 Hz WS state push (#124)
// so the hot path stays small and never touches the filesystem. Each helper is
// self-guarding and always returns a valid JSON object (never an empty string).
static String build_aux_json() {
    String j;
    j.reserve(1536);
    j  = "{";
    j += "\"blackbox\":";   j += blackbox_status_json();   j += ',';
    j += "\"capability\":"; j += capability_status_json(); j += ',';
    j += "\"profile\":";    j += profile_match_json();
    j += '}';
    return j;
}

// ── WebSocket event handler ───────────────────────────────────────────────────
static void ws_event(uint8_t num, WStype_t type,
                     uint8_t *payload, size_t length)
{
    if (type == WStype_CONNECTED) {
        // Auto-run the tap capability check on connect (#125): the first few
        // seconds answer "will the nag killer work on this tap?" before any
        // guesswork. Pure RX — counting only.
        capability_start(millis());
        // Push current state immediately on connect
        String json = build_json();
        g_ws.sendTXT(num, json.c_str(), json.length());
        return;
    }

    if (type != WStype_TEXT || g_state == nullptr || length == 0) return;

    // Use a slightly more robust way to find the value after the second colon
    char buf[256] = {};
    size_t n = (length < sizeof(buf) - 1) ? length : sizeof(buf) - 1;
    memcpy(buf, payload, n);

    // Find the "value" part of {"cmd":"xxx","value":yyy}
    const char *vptr = strstr(buf, "\"value\":");
    if (vptr) vptr = strstr(vptr, ":") + 1;

    if (strstr(buf, "\"mode\"")) {
        FSDState saved;
        bool active = false;
        state_enter();
        if (g_state->op_mode == OpMode_ListenOnly) {
            g_state->op_mode = OpMode_Active;
            active = true;
        } else {
            g_state->op_mode = OpMode_ListenOnly;
        }
        saved = *g_state;
        state_exit();
        for (uint8_t i = 0; i < g_can_count; i++) {
            if (g_can_buses[i]) g_can_buses[i]->setListenOnly(!active);
        }
        http_can_stream_set_enabled(true);  // capture works in both modes now (#108)
        Serial.println(active ? "[Web] → Active mode" : "[Web] → Listen-Only mode");
        prefs_save(&saved);
    } else if (strstr(buf, "\"hw_override\"")) {
        // Manual HW selection (#110): 0 = auto-detect, else pin the version.
        if (vptr) {
            while (*vptr == ' ' || *vptr == ':') vptr++;
            int sel = atoi(vptr);
            if (sel >= (int)TeslaHW_Unknown && sel <= (int)TeslaHW_HW4) {
                TeslaHWVersion want = (TeslaHWVersion)sel;
                FSDState saved;
                state_enter();
                g_state->hw_override = want;
                // Apply at once so the right handlers are live immediately; on
                // "auto" leave the current detection in place and let the normal
                // detectors take over again from the next frames.
                if (want != TeslaHW_Unknown) fsd_apply_hw_version(g_state, want);
                saved = *g_state;
                state_exit();
                Serial.printf("[Web] HW override: %s\n",
                              (want == TeslaHW_HW4)    ? "HW4" :
                              (want == TeslaHW_HW3)    ? "HW3" :
                              (want == TeslaHW_Legacy) ? "Legacy" : "Auto");
                prefs_save(&saved);
            }
        }
    } else if (strstr(buf, "\"fsd_protocol\"")) {
        if (vptr) {
            while (*vptr == ' ' || *vptr == ':') vptr++;
            int sel = atoi(vptr);
            if (sel != 13 && sel != 14) sel = 0;
            FSDState saved;
            state_enter();
            g_state->fsd_protocol_mode = (uint8_t)sel;
            // Re-evaluate profile locking without changing physical HW parsing.
            if (sel == 13 && g_state->hw3_drive_style >= 1 && g_state->hw3_drive_style <= 3) {
                g_state->speed_profile_locked = true;
                g_state->speed_profile = (int)g_state->hw3_drive_style - 1;
            } else if (sel == 14) {
                g_state->speed_profile_locked = false;
                if (g_state->speed_profile < 0 || g_state->speed_profile > 4) g_state->speed_profile = 4;
            }
            saved = *g_state;
            state_exit();
            prefs_save(&saved);
        }
    } else if (strstr(buf, "\"drive_style\"")) {
        if (vptr) {
            while (*vptr == ' ' || *vptr == ':') vptr++;
            int sel = atoi(vptr);
            if (sel < 0) sel = 0;
            if (sel > 5) sel = 5;
            FSDState saved;
            state_enter();
            g_state->hw3_drive_style = (uint8_t)sel;
            if (sel == 0) {
                g_state->speed_profile_locked = false;
            } else {
                bool v13 = (g_state->fsd_protocol_mode == 13) ||
                           (g_state->fsd_protocol_mode == 0 &&
                            (g_state->hw_version == TeslaHW_HW3 ||
                             g_state->hw_override == TeslaHW_HW3));
                if (v13) {
                    g_state->speed_profile_locked = true;
                    g_state->speed_profile = sel - 1;
                }
            }
            saved = *g_state;
            state_exit();
            prefs_save(&saved);
        }
    } else if (strstr(buf, "\"hw3_offset_mode\"") || strstr(buf, "\"hw3_manual_offset\"") ||
               strstr(buf, "\"hw3_cp0\"") || strstr(buf, "\"hw3_cp1\"") ||
               strstr(buf, "\"hw3_cp2\"") || strstr(buf, "\"hw3_cp3\"")) {
        if (vptr) {
            while (*vptr == ' ' || *vptr == ':') vptr++;
            bool bv = (strncmp(vptr, "true", 4) == 0);
            int iv = atoi(vptr);
            FSDState saved;
            state_enter();
            if (strstr(buf, "\"hw3_offset_mode\"")) g_state->hw3_offset_mode=(uint8_t)((iv<0)?0:(iv>2?2:iv));
            else if (strstr(buf, "\"hw3_manual_offset\"")) g_state->hw3_manual_offset=(uint8_t)((iv<0)?0:(iv>63?63:iv));
            else if (strstr(buf, "\"hw3_cp0\"")) g_state->hw3_custom_pct[0]=(uint8_t)((iv<0)?0:(iv>63?63:iv));
            else if (strstr(buf, "\"hw3_cp1\"")) g_state->hw3_custom_pct[1]=(uint8_t)((iv<0)?0:(iv>63?63:iv));
            else if (strstr(buf, "\"hw3_cp2\"")) g_state->hw3_custom_pct[2]=(uint8_t)((iv<0)?0:(iv>63?63:iv));
            else if (strstr(buf, "\"hw3_cp3\"")) g_state->hw3_custom_pct[3]=(uint8_t)((iv<0)?0:(iv>63?63:iv));
            saved=*g_state;
            state_exit();
            prefs_save(&saved);
        }
    } else if (strstr(buf, "\"ignore_ota\"")) {
        if (vptr) {
            while (*vptr == ' ' || *vptr == ':') vptr++;
            bool enabled = (strncmp(vptr, "true", 4) == 0);
            FSDState saved;
            state_enter();
            g_state->ignore_ota = enabled;
            saved = *g_state;
            state_exit();
            Serial.printf("[Web] Ignore OTA: %s\n", enabled ? "ON" : "OFF");
            prefs_save(&saved);
        }
    } else if (strstr(buf, "\"fsd_unlock\"")) {
        if (vptr) {
            while (*vptr == ' ' || *vptr == ':') vptr++;
            bool enabled = (strncmp(vptr, "true", 4) == 0);
            FSDState saved;
            state_enter();
            g_state->fsd_unlock = enabled;
            saved = *g_state;
            state_exit();
            Serial.printf("[Web] FSD Unlock: %s\n", enabled ? "ON" : "OFF");
            prefs_save(&saved);
        }
    } else if (strstr(buf, "\"nag\"")) {
        if (vptr) {
            while (*vptr == ' ' || *vptr == ':') vptr++;
            bool enabled = (strncmp(vptr, "true", 4) == 0);
            FSDState saved;
            state_enter();
            g_state->nag_killer = enabled;
            saved = *g_state;
            state_exit();
            Serial.printf("[Web] NAG Killer: %s\n", enabled ? "ON" : "OFF");
            prefs_save(&saved);
        }
    } else if (strstr(buf, "\"continuous_ap\"")) {
        if (vptr) {
            while (*vptr == ' ' || *vptr == ':') vptr++;
            bool enabled = (strncmp(vptr, "true", 4) == 0);
            FSDState saved;
            state_enter();
            g_state->continuous_ap = enabled;
            saved = *g_state;
            state_exit();
            Serial.printf("[Web] Continuous AP: %s\n", enabled ? "ON" : "OFF");
            prefs_save(&saved);
        }
    } else if (strstr(buf, "\"ap_first\"")) {
        if (vptr) {
            while (*vptr == ' ' || *vptr == ':') vptr++;
            bool enabled = (strncmp(vptr, "true", 4) == 0);
            FSDState saved;
            state_enter();
            g_state->ap_first = enabled;
            saved = *g_state;
            state_exit();
            Serial.printf("[Web] AP-First: %s\n", enabled ? "ON" : "OFF");
            prefs_save(&saved);
        }
    } else if (strstr(buf, "\"ap_first_edge\"")) {
        if (vptr) {
            while (*vptr == ' ' || *vptr == ':') vptr++;
            bool enabled = (strncmp(vptr, "true", 4) == 0);
            FSDState saved;
            state_enter();
            g_state->ap_first_edge = enabled;
            saved = *g_state;
            state_exit();
            Serial.printf("[Web] Instant Engage: %s\n", enabled ? "ON" : "OFF");
            prefs_save(&saved);
        }
    } else if (strstr(buf, "\"ap_first_minimal\"")) {
        if (vptr) {
            while (*vptr == ' ' || *vptr == ':') vptr++;
            bool enabled = (strncmp(vptr, "true", 4) == 0);
            FSDState saved;
            state_enter();
            g_state->ap_first_minimal = enabled;
            saved = *g_state;
            state_exit();
            Serial.printf("[Web] Minimal Inject: %s\n", enabled ? "ON" : "OFF");
            prefs_save(&saved);
        }
    } else if (strstr(buf, "\"nag_faithful\"")) {
        if (vptr) {
            while (*vptr == ' ' || *vptr == ':') vptr++;
            bool enabled = (strncmp(vptr, "true", 4) == 0);
            FSDState saved;
            state_enter();
            g_state->nag_epas_faithful = enabled;
            saved = *g_state;
            state_exit();
            Serial.printf("[Web] Nag EPAS-faithful: %s\n", enabled ? "ON" : "OFF");
            prefs_save(&saved);
        }
    } else if (strstr(buf, "\"soft_engage\"")) {
        if (vptr) {
            while (*vptr == ' ' || *vptr == ':') vptr++;
            bool enabled = (strncmp(vptr, "true", 4) == 0);
            FSDState saved;
            state_enter();
            g_state->soft_engage = enabled;
            saved = *g_state;
            state_exit();
            Serial.printf("[Web] Soft Engage: %s\n", enabled ? "ON" : "OFF");
            prefs_save(&saved);
        }
    } else if (strstr(buf, "\"nag_burst\"")) {
        if (vptr) {
            while (*vptr == ' ' || *vptr == ':') vptr++;
            bool enabled = (strncmp(vptr, "true", 4) == 0);
            FSDState saved;
            state_enter();
            g_state->nag_burst = enabled;
            saved = *g_state;
            state_exit();
            Serial.printf("[Web] Nag Burst: %s\n", enabled ? "ON" : "OFF");
            prefs_save(&saved);
        }
    } else if (strstr(buf, "\"abort_guard\"")) {
        if (vptr) {
            while (*vptr == ' ' || *vptr == ':') vptr++;
            bool enabled = (strncmp(vptr, "true", 4) == 0);
            FSDState saved;
            state_enter();
            g_state->abort_guard = enabled;
            if (!enabled) g_state->abort_guard_latched = false;
            saved = *g_state;
            state_exit();
            Serial.printf("[Web] Abort Guard: %s\n", enabled ? "ON" : "OFF");
            prefs_save(&saved);
        }
    } else if (strstr(buf, "\"sig_cfg\"")) {
        // value is a 10-field CSV: das_id,apB,apS,apM,hoB,hoS,hoM,steer_id,sHi,sLo
        // ids accept 0x.. ; byte/shift 0-7, mask hex. (#122)
        if (vptr) {
            while (*vptr == ' ' || *vptr == ':' || *vptr == '"') vptr++;
            char csv[128] = {};
            size_t ci = 0;
            for (const char *p = vptr; *p && *p != '"' && *p != '}' && ci < sizeof(csv) - 1; p++)
                csv[ci++] = *p;
            long v[10]; int got = 0;
            char *tok = strtok(csv, ",");
            while (tok && got < 10) { v[got++] = strtol(tok, NULL, 0); tok = strtok(NULL, ","); }
            if (got == 10) {
                FSDState saved;
                state_enter();
                g_state->cfg_das_id        = (uint16_t)v[0];
                g_state->cfg_apstate_byte  = (uint8_t)(v[1] & 7);
                g_state->cfg_apstate_shift = (uint8_t)(v[2] & 7);
                g_state->cfg_apstate_mask  = (uint8_t)v[3];
                g_state->cfg_handson_byte  = (uint8_t)(v[4] & 7);
                g_state->cfg_handson_shift = (uint8_t)(v[5] & 7);
                g_state->cfg_handson_mask  = (uint8_t)v[6];
                g_state->cfg_steer_id      = (uint16_t)v[7];
                g_state->cfg_steer_hi      = (uint8_t)(v[8] & 7);
                g_state->cfg_steer_lo      = (uint8_t)(v[9] & 7);
                saved = *g_state;
                state_exit();
                Serial.printf("[Web] Signal map: das=0x%X ap=%ld/%ld/0x%lX ho=%ld/%ld/0x%lX steer=0x%X %ld/%ld\n",
                              (uint16_t)v[0], v[1], v[2], v[3], v[4], v[5], v[6],
                              (uint16_t)v[7], v[8], v[9]);
                prefs_save(&saved);
            }
        }
    } else if (strstr(buf, "\"blackbox_enable\"")) {
        if (vptr) {
            while (*vptr == ' ' || *vptr == ':') vptr++;
            bool enabled = (strncmp(vptr, "true", 4) == 0);
            blackbox_set_enabled(enabled);          // locks the state mux itself
            FSDState saved;
            state_enter();
            saved = *g_state;
            state_exit();
            Serial.printf("[Web] Black-box: %s\n", enabled ? "ON" : "OFF");
            prefs_save(&saved);
        }
    } else if (strstr(buf, "\"capability_recheck\"")) {
        capability_start(millis());                 // re-run the tap check (#125)
        Serial.println("[Web] Capability: re-check");
    } else if (strstr(buf, "\"blackbox_mark\"")) {
        blackbox_mark(millis());                    // inject EVT_MANUAL + arm
        Serial.println("[Web] Black-box: manual mark");
    } else if (strstr(buf, "\"blackbox_delete_all\"")) {
        blackbox_delete_all();
        Serial.println("[Web] Black-box: delete all");
    } else if (strstr(buf, "\"blackbox_delete\"")) {
        const char *v = strstr(buf, "\"value\":\"");
        if (v) {
            v += 9;
            char name[40];
            size_t i = 0;
            while (v[i] && v[i] != '\"' && i < sizeof(name) - 1) { name[i] = v[i]; i++; }
            name[i] = '\0';
            if (name[0]) { blackbox_delete(name); Serial.printf("[Web] Black-box: delete %s\n", name); }
        }
    }
#if defined(BOARD_TTGO_DISPLAY)
    else if (strstr(buf, "\"disp\"")) {
        if (vptr) {
            while (*vptr == ' ' || *vptr == ':') vptr++;
            bool enabled = (strncmp(vptr, "true", 4) == 0);
            FSDState saved;
            state_enter();
            g_state->display_enabled = enabled;
            saved = *g_state;
            state_exit();
            Serial.printf("[Web] Display: %s\n", enabled ? "ON" : "OFF");
            prefs_save(&saved);
        }
    } else if (strstr(buf, "\"disp_br\"")) {
        if (vptr) {
            while (*vptr == ' ' || *vptr == ':') vptr++;
            uint8_t val = (uint8_t)atoi(vptr);
            if (val > 100) val = 100;
            FSDState saved;
            state_enter();
            g_state->display_brightness = val;
            saved = *g_state;
            state_exit();
            Serial.printf("[Web] Display Brightness: %u\n", val);
            prefs_save(&saved);
        }
    } else if (strstr(buf, "\"disp_to\"")) {
        if (vptr) {
            while (*vptr == ' ' || *vptr == ':') vptr++;
            uint32_t val = (uint32_t)atoi(vptr);
            FSDState saved;
            state_enter();
            g_state->display_timeout_s = val;
            saved = *g_state;
            state_exit();
            Serial.printf("[Web] Display Timeout: %u s\n", val);
            prefs_save(&saved);
        }
    }
#endif
    else if (strstr(buf, "\"bms\"")) {
        if (vptr) {
            while (*vptr == ' ' || *vptr == ':') vptr++;
            bool enabled = (strncmp(vptr, "true", 4) == 0);
            FSDState saved;
            state_enter();
            g_state->bms_output = enabled;
            saved = *g_state;
            state_exit();
            Serial.printf("[Web] BMS output: %s\n", enabled ? "ON" : "OFF");
            prefs_save(&saved);
        }
    } else if (strstr(buf, "\"tlssc_restore\"")) {
        if (vptr) {
            while (*vptr == ' ' || *vptr == ':') vptr++;
            bool enabled = (strncmp(vptr, "true", 4) == 0);
            FSDState saved;
            state_enter();
            g_state->tlssc_restore = enabled;
            saved = *g_state;
            state_exit();
            Serial.printf("[Web] TLSSC Restore: %s\n", enabled ? "ON" : "OFF");
            prefs_save(&saved);
        }
    } else if (strstr(buf, "\"14x_warning\"")) {
        if (vptr) {
            while (*vptr == ' ' || *vptr == ':') vptr++;
            bool enabled = (strncmp(vptr, "true", 4) == 0);
            FSDState saved;
            state_enter();
            g_state->firmware_14x_warning = enabled;
            saved = *g_state;
            state_exit();
            Serial.printf("[Web] 14.x Warning: %s\n", enabled ? "ON" : "OFF");
            prefs_save(&saved);
        }
    } else if (strstr(buf, "\"force_fsd\"")) {
        if (vptr) {
            while (*vptr == ' ' || *vptr == ':') vptr++;
            bool enabled = (strncmp(vptr, "true", 4) == 0);
            FSDState saved;
            state_enter();
            g_state->force_fsd = enabled;
            saved = *g_state;
            state_exit();
            Serial.printf("[Web] Force FSD: %s\n", enabled ? "ON" : "OFF");
            prefs_save(&saved);
        }
    } else if (strstr(buf, "\"china_mode\"")) {
        if (vptr) {
            while (*vptr == ' ' || *vptr == ':') vptr++;
            bool enabled = (strncmp(vptr, "true", 4) == 0);
            FSDState saved;
            state_enter();
            g_state->china_mode = enabled;
            saved = *g_state;
            state_exit();
            Serial.printf("[Web] China Mode: %s\n", enabled ? "ON" : "OFF");
            prefs_save(&saved);
        }
    } else if (strstr(buf, "\"suppress_speed_chime\"")) {
        if (vptr) {
            while (*vptr == ' ' || *vptr == ':') vptr++;
            bool enabled = (strncmp(vptr, "true", 4) == 0);
            FSDState saved;
            state_enter();
            g_state->suppress_speed_chime = enabled;
            saved = *g_state;
            state_exit();
            Serial.printf("[Web] Suppress Speed Chime: %s\n", enabled ? "ON" : "OFF");
            prefs_save(&saved);
        }
    } else if (strstr(buf, "\"summon_unlock\"")) {
        if (vptr) {
            while (*vptr == ' ' || *vptr == ':') vptr++;
            bool enabled = (strncmp(vptr, "true", 4) == 0);
            FSDState saved;
            state_enter();
            g_state->summon_unlock = enabled;
            saved = *g_state;
            state_exit();
            Serial.printf("[Web] Summon EU Unlock: %s\n", enabled ? "ON" : "OFF");
            prefs_save(&saved);
        }
    } else if (strstr(buf, "\"continue_on_green\"")) {
        if (vptr) {
            while (*vptr == ' ' || *vptr == ':') vptr++;
            bool enabled = (strncmp(vptr, "true", 4) == 0);
            FSDState saved;
            state_enter();
            g_state->continue_on_green = enabled;
            saved = *g_state;
            state_exit();
            Serial.printf("[Web] Continue on Green: %s\n", enabled ? "ON" : "OFF");
            prefs_save(&saved);
        }
    } else if (strstr(buf, "\"assist_tlssc_bit38\"")) {
        if (vptr) {
            while (*vptr == ' ' || *vptr == ':') vptr++;
            bool enabled = (strncmp(vptr, "true", 4) == 0);
            FSDState saved;
            state_enter();
            g_state->assist_tlssc_bit38 = enabled;
            saved = *g_state;
            state_exit();
            Serial.printf("[Web] TLSSC bit38: %s\n", enabled ? "ON" : "OFF");
            prefs_save(&saved);
        }
    } else if (strstr(buf, "\"assist_rhd_override\"")) {
        if (vptr) {
            while (*vptr == ' ' || *vptr == ':') vptr++;
            bool enabled = (strncmp(vptr, "true", 4) == 0);
            FSDState saved;
            state_enter();
            g_state->assist_rhd_override = enabled;
            saved = *g_state;
            state_exit();
            Serial.printf("[Web] RHD Override: %s\n", enabled ? "ON" : "OFF");
            prefs_save(&saved);
        }
    } else if (strstr(buf, "\"assist_telemetry_off\"")) {
        if (vptr) {
            while (*vptr == ' ' || *vptr == ':') vptr++;
            bool enabled = (strncmp(vptr, "true", 4) == 0);
            FSDState saved;
            state_enter();
            g_state->assist_telemetry_off = enabled;
            saved = *g_state;
            state_exit();
            Serial.printf("[Web] Telemetry Off: %s\n", enabled ? "ON" : "OFF");
            prefs_save(&saved);
        }
    } else if (strstr(buf, "\"apmv3_branch\"")) {
        // AP branch/tier selector (experimental, non-persistent): 0-5 select a
        // UI_apmv3Branch value, any other value (255 = Off) stores the 0xFF
        // sentinel so the handler leaves the frame untouched.
        if (vptr) {
            while (*vptr == ' ' || *vptr == ':') vptr++;
            int sel = atoi(vptr);
            uint8_t want = (sel >= 0 && sel <= 5) ? (uint8_t)sel : 0xFF;
            FSDState saved;
            state_enter();
            g_state->apmv3_branch = want;
            saved = *g_state;
            state_exit();
            Serial.printf("[Web] AP Branch/Tier: %d\n", want);
            prefs_save(&saved);
        }
    } else if (strstr(buf, "\"track_mode_inject\"")) {
        if (vptr) {
            while (*vptr == ' ' || *vptr == ':') vptr++;
            bool enabled = (strncmp(vptr, "true", 4) == 0);
            FSDState saved;
            state_enter();
            g_state->track_mode_inject = enabled;
            saved = *g_state;
            state_exit();
            Serial.printf("[Web] Track Mode inject: %s\n", enabled ? "ON" : "OFF");
            prefs_save(&saved);
        }
    } else if (strstr(buf, "\"track_rotation_pct\"")) {
        if (vptr) {
            while (*vptr == ' ' || *vptr == ':') vptr++;
            uint8_t val = (uint8_t)atoi(vptr);
            if (val > 100) val = 100;
            FSDState saved;
            state_enter();
            g_state->track_rotation_pct = val;
            saved = *g_state;
            state_exit();
            Serial.printf("[Web] Track Handling Balance: %u\n", val);
            prefs_save(&saved);
        }
    } else if (strstr(buf, "\"track_stability_pct\"")) {
        if (vptr) {
            while (*vptr == ' ' || *vptr == ':') vptr++;
            uint8_t val = (uint8_t)atoi(vptr);
            if (val > 100) val = 100;
            FSDState saved;
            state_enter();
            g_state->track_stability_pct = val;
            saved = *g_state;
            state_exit();
            Serial.printf("[Web] Track Stability Assist: %u\n", val);
            prefs_save(&saved);
        }
    } else if (strstr(buf, "\"track_post_cooling\"")) {
        if (vptr) {
            while (*vptr == ' ' || *vptr == ':') vptr++;
            bool enabled = (strncmp(vptr, "true", 4) == 0);
            FSDState saved;
            state_enter();
            g_state->track_post_cooling = enabled;
            saved = *g_state;
            state_exit();
            Serial.printf("[Web] Track Post-drive Cooling: %s\n", enabled ? "ON" : "OFF");
            prefs_save(&saved);
        }
    } else if (strstr(buf, "\"track_cmp_overclock\"")) {
        if (vptr) {
            while (*vptr == ' ' || *vptr == ':') vptr++;
            bool enabled = (strncmp(vptr, "true", 4) == 0);
            FSDState saved;
            state_enter();
            g_state->track_cmp_overclock = enabled;
            saved = *g_state;
            state_exit();
            Serial.printf("[Web] Track Compressor Overclock: %s\n", enabled ? "ON" : "OFF");
            prefs_save(&saved);
        }
    } else if (strstr(buf, "\"dump\"")) {
        if (vptr) {
            while (*vptr == ' ' || *vptr == ':') vptr++;
            bool want = (strncmp(vptr, "true", 4) == 0);
            if (want) can_dump_start();
            else      can_dump_stop();
            Serial.printf("[Web] CAN Dump: %s\n", want ? "START" : "STOP");
        }
    } else if (strstr(buf, "\"sleep\"")) {
        if (vptr) {
            while (*vptr == ' ' || *vptr == ':') vptr++;
            uint32_t val = (uint32_t)atoi(vptr);
            if (val >= 10000) { // minimum 10s
                FSDState saved;
                state_enter();
                g_state->sleep_idle_ms = val;
                saved = *g_state;
                state_exit();
                Serial.printf("[Web] Sleep timeout: %u ms\n", val);
                prefs_save(&saved);
            }
        }
    } else if (strstr(buf, "\"wifi_cfg\"")) {
        // Find the "value":{ object start
        const char *vobj = strstr(buf, "\"value\":");
        if (vobj) {
            FSDState saved;
            state_enter();
            char *s = strstr(vobj, "\"ssid\":\"");
            char *p = strstr(vobj, "\"pass\":\"");
            char *h = strstr(vobj, "\"hidden\":");
            char *ss = strstr(vobj, "\"sta_ssid\":\"");
            char *sp = strstr(vobj, "\"sta_pass\":\"");
            if (s) {
                s += 8;
                char *end = strchr(s, '\"');
                if (end) {
                    int len = end - s;
                    if (len > 32) len = 32;
                    if (memchr(s, '\\', len) == nullptr) {
                        memcpy(g_state->wifi_ssid, s, len);
                        g_state->wifi_ssid[len] = '\0';
                    }
                }
            }
            if (p) {
                p += 8;
                char *end = strchr(p, '\"');
                if (end) {
                    int len = end - p;
                    if (len > 64) len = 64;
                    if (memchr(p, '\\', len) == nullptr &&
                        !(len == 3 && memcmp(p, "***", 3) == 0)) {
                        memcpy(g_state->wifi_pass, p, len);
                        g_state->wifi_pass[len] = '\0';
                    }
                }
            }
            if (h) {
                h += 9;
                while (*h == ' ' || *h == ':') h++;
                if (strncmp(h, "true", 4) == 0) g_state->wifi_hidden = true;
                else if (strncmp(h, "false", 5) == 0) g_state->wifi_hidden = false;
            }
            if (ss) {
                ss += 12;
                char *end = strchr(ss, '\"');
                if (end) {
                    int len = end - ss;
                    if (len > 32) len = 32;
                    if (memchr(ss, '\\', len) == nullptr) {
                        memcpy(g_state->wifi_sta_ssid, ss, len);
                        g_state->wifi_sta_ssid[len] = '\0';
                    }
                }
            }
            if (sp) {
                sp += 12;
                char *end = strchr(sp, '\"');
                if (end) {
                    int len = end - sp;
                    if (len > 64) len = 64;
                    if (memchr(sp, '\\', len) == nullptr &&
                        !(len == 3 && memcmp(sp, "***", 3) == 0)) {
                        memcpy(g_state->wifi_sta_pass, sp, len);
                        g_state->wifi_sta_pass[len] = '\0';
                    }
                }
            }
            saved = *g_state;
            state_exit();
            Serial.printf("[Web] WiFi config: AP=\"%s\" STA=\"%s\" PASS=*** HIDDEN=%d\n",
                saved.wifi_ssid, saved.wifi_sta_ssid, saved.wifi_hidden);
            prefs_save(&saved);
            can_shutdown_all(g_can_buses, g_can_count);
            delay(500);
            ESP.restart();
        }
    }
}

// ── HTTP handlers ─────────────────────────────────────────────────────────────
static void handle_root() {
    g_http.setContentLength(sizeof(WEB_HTML) - 1);
    g_http.send(200, "text/html", "");

    const uint8_t *ptr = (const uint8_t *)WEB_HTML;
    size_t left = sizeof(WEB_HTML) - 1;
    WiFiClient client = g_http.client();

    uint32_t timeout_ms = millis();
    while (left > 0 && client.connected()) {
        size_t chunk = (left > 1460) ? 1460 : left;
        size_t written = client.write(ptr, chunk);
        if (written > 0) {
            ptr += written;
            left -= written;
            timeout_ms = millis(); // Reset timeout
        } else {
            if (millis() - timeout_ms > 2000) break; // Prevent infinite loop
            delay(10);
        }
        delay(2);
    }
}

static void handle_status() {
    if (g_state == nullptr) { g_http.send(503, "application/json", "{}"); return; }
    g_http.send(200, "application/json", build_json());
}

// On-demand aux status (blackbox/capability/profile). Polled by the dashboard
// on a slow timer so these heavy blocks stay off the hot WS state push (#124).
static void handle_aux() {
    if (g_state == nullptr) { g_http.send(503, "application/json", "{}"); return; }
    g_http.sendHeader("Cache-Control", "no-store");
    g_http.send(200, "application/json", build_aux_json());
}

static void handle_auth() {
    if (!require_admin_auth()) return;
    g_http.send(200, "text/plain", "OK");
}

static void handle_sdformat() {
    String result = sd_format_card();
    g_http.send(200, "application/json", result);
}

// ── Black-box (#124) ──────────────────────────────────────────────────────────
static void handle_blackbox_list() {
    if (!download_auth_ok()) return;
    g_http.sendHeader("Cache-Control", "no-store");
    g_http.send(200, "application/json", blackbox_list_json());
}

static void handle_blackbox_get() {
    if (!download_auth_ok()) return;
    String name = g_http.arg("name");
    bool json = (g_http.arg("type") == "json");
    if (name.length() == 0 || name.length() >= 40) {
        g_http.send(400, "text/plain", "bad name");
        return;
    }
    size_t size = 0;
    if (!blackbox_file_size(name.c_str(), json, &size)) {
        g_http.send(404, "text/plain", "no such event");
        return;
    }
    String fname = name + (json ? ".json" : ".log");
    g_http.setContentLength(size);
    g_http.sendHeader("Content-Disposition", "attachment; filename=\"" + fname + "\"");
    g_http.sendHeader("Cache-Control", "no-store");
    g_http.send(200, json ? "application/json" : "text/plain", "");
    WiFiClient client = g_http.client();
    blackbox_stream_body(client, name.c_str(), json);
    client.flush();
}

static void handle_restart() {
    if (!require_admin_auth()) return;
    g_http.send(200, "text/plain", "OK");
    can_shutdown_all(g_can_buses, g_can_count);
    delay(500);
    ESP.restart();
}

// ── OTA Update handlers ───────────────────────────────────────────────────────
static size_t ota_total_size = 0;
static size_t ota_max_size = 0;
static bool ota_error_flag = false;
static bool ota_magic_checked = false;
static const char *ota_error_msg = nullptr;

static void handle_ota_upload() {
    HTTPUpload& upload = g_http.upload();

    if (upload.status == UPLOAD_FILE_START) {
        Serial.printf("[OTA] Start: %s\n", upload.filename.c_str());
        if (!require_admin_auth()) {
            ota_error_flag = true;
            ota_error_msg = "Authentication required";
            return;
        }
        ota_error_flag = false;
        ota_error_msg = nullptr;
        ota_magic_checked = false;
        ota_total_size = 0;
        ota_max_size = 0;

        if (!upload.filename.endsWith(".bin")) {
            Serial.println("[OTA] ERROR: File must be .bin");
            ota_error_flag = true;
            ota_error_msg = "File must be .bin";
            return;
        }

        size_t max_size = UPDATE_SIZE_UNKNOWN;
        const esp_partition_t* partition = esp_ota_get_next_update_partition(NULL);
        if (partition != NULL) {
            max_size = partition->size;
            ota_max_size = partition->size;
            Serial.printf("[OTA] Target partition: %s, size: %u bytes\n",
                partition->label, (unsigned)max_size);
        } else {
            Serial.println("[OTA] ERROR: No OTA partition available");
            ota_error_flag = true;
            ota_error_msg = "No OTA partition available";
            return;
        }

        if (!Update.begin(max_size, U_FLASH)) {
            Update.printError(Serial);
            Serial.println("[OTA] ERROR: Update.begin() failed");
            ota_error_flag = true;
            ota_error_msg = "Update.begin() failed";
            return;
        }

        Serial.println("[OTA] Update started successfully");
    } else if (upload.status == UPLOAD_FILE_WRITE) {
        if (ota_error_flag) return;
        if (!ota_magic_checked) {
            if (upload.currentSize == 0 || upload.buf[0] != OTA_ESP32_IMAGE_MAGIC) {
                Serial.println("[OTA] ERROR: Invalid ESP32 image magic byte");
                ota_error_flag = true;
                ota_error_msg = "Invalid ESP32 image magic byte";
                Update.abort();
                return;
            }
            ota_magic_checked = true;
        }
        if (ota_max_size > 0 && (ota_total_size + upload.currentSize) > ota_max_size) {
            Serial.println("[OTA] ERROR: Firmware exceeds OTA partition size");
            ota_error_flag = true;
            ota_error_msg = "Firmware exceeds OTA partition size";
            Update.abort();
            return;
        }

        size_t written = Update.write(upload.buf, upload.currentSize);
        if (written != upload.currentSize) {
            Update.printError(Serial);
            Serial.printf("[OTA] ERROR: Write failed, expected %u, wrote %u\n",
                upload.currentSize, (unsigned)written);
            ota_error_flag = true;
            ota_error_msg = "Flash write failed";
            return;
        }

        ota_total_size += upload.currentSize;
        if (ota_total_size % 65536 == 0) {
            Serial.printf("[OTA] Progress: %u bytes\n", (unsigned)ota_total_size);
        }
    } else if (upload.status == UPLOAD_FILE_END) {
        if (ota_error_flag) {
            Serial.println("[OTA] Upload aborted due to previous error");
            Update.abort();
            return;
        }

        if (Update.end(true)) {
            Serial.printf("[OTA] Success: %u bytes total\n", (unsigned)ota_total_size);
            if (!Update.isFinished()) {
                Serial.println("[OTA] ERROR: Update not finished properly");
                ota_error_flag = true;
                ota_error_msg = "Update not finished properly";
            }
        } else {
            Update.printError(Serial);
            Serial.println("[OTA] ERROR: Update.end() failed");
            ota_error_flag = true;
            ota_error_msg = "Update.end() failed";
        }
    } else if (upload.status == UPLOAD_FILE_ABORTED) {
        Serial.println("[OTA] Upload aborted by client");
        Update.abort();
        ota_error_flag = true;
        ota_error_msg = "Upload aborted";
    }
}

static void handle_ota_done() {
    if (!require_admin_auth()) return;
    if (ota_error_flag || Update.hasError()) {
        String error_msg = "FAIL: ";
        if (Update.hasError()) {
            error_msg += "Error code " + String(Update.getError());
        } else if (ota_error_msg != nullptr) {
            error_msg += ota_error_msg;
        } else {
            error_msg += "Upload error";
        }

        Serial.printf("[OTA] %s\n", error_msg.c_str());
        g_http.send(500, "text/plain", error_msg);
        Update.abort();
        return;
    }

    g_http.send(200, "text/plain", "OK");

    Serial.println("[OTA] Firmware update successful!");
    Serial.println("[OTA] Rebooting in 2 seconds...");

    can_shutdown_all(g_can_buses, g_can_count);
    delay(2000);
    ESP.restart();
}

// ── Public API ────────────────────────────────────────────────────────────────
void web_dashboard_init(FSDState *state,
                        CanDriver **can_buses,
                        uint8_t can_count,
                        portMUX_TYPE *state_mux) {
    g_state       = state;
    g_can_buses   = can_buses;
    g_can_count   = can_count;
    g_state_mux   = state_mux;
    g_start_ms    = millis();
    g_last_fps_ms = millis();
    g_last_rx     = state ? state->rx_count : 0;
    g_last_can_seen_ms = (state && state->rx_count > 0) ? millis() : 0;
    sample_chip_temperature();

    g_http.on("/",           HTTP_GET,  handle_root);
    g_http.on("/api/status", HTTP_GET,  handle_status);
    g_http.on("/api/aux",    HTTP_GET,  handle_aux);
    g_http.on("/auth",       HTTP_GET,  handle_auth);
    g_http.on("/sdformat",   HTTP_GET,  handle_sdformat);
    g_http.on("/blackbox/list", HTTP_GET, handle_blackbox_list);
    g_http.on("/blackbox/get",  HTTP_GET, handle_blackbox_get);
    g_http.on("/restart",    HTTP_GET,  handle_restart);
    g_http.on("/update",     HTTP_POST, handle_ota_done, handle_ota_upload);
    g_http.begin();
    http_can_stream_init();

    g_ws.begin();
    g_ws.onEvent(ws_event);

    Serial.println("[Web] HTTP :80  WS :81 — ready");
}

void web_dashboard_update() {
    if (g_state == nullptr) return;   // init was never called (WiFi failed)

    g_http.handleClient();
    g_ws.loop();
    http_can_stream_update();

    // FPS calculation + 1 Hz WebSocket broadcast
    uint32_t now = millis();
    if ((now - g_last_fps_ms) >= 1000u) {
        sample_chip_temperature();
        FSDState state;
        if (!state_copy(&state)) return;
        uint32_t rx = state.rx_count;
        float    dt = (now - g_last_fps_ms) / 1000.0f;
        if (rx != g_last_rx) g_last_can_seen_ms = now;
        g_fps        = (float)(rx - g_last_rx) / dt;
        g_last_rx    = rx;
        g_last_fps_ms = now;

        String json = build_json();
        Serial.printf("[WS] state json=%u bytes\n", (unsigned)json.length());
        g_ws.broadcastTXT(json.c_str(), json.length());
    }
}
