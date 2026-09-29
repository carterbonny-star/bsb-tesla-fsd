#include "prefs.h"
#include "blackbox.h"   // BLACKBOX_DEFAULT_ENABLED
#include <Preferences.h>

static Preferences g_prefs;
static const char *NS = "fsd";

void prefs_load(FSDState *state) {
    g_prefs.begin(NS, /*readOnly=*/true);
    if (!g_prefs.isKey("ok")) {
        Serial.println("[NVS] No saved settings found (first boot)");
        g_prefs.end();
        return;
    }
    state->nag_killer               = g_prefs.getBool("nag",    true);
    state->continuous_ap            = g_prefs.getBool("contap", false);
    state->ap_first                 = g_prefs.getBool("apfirst",false);
    state->ap_first_edge            = g_prefs.getBool("apfe",   false);
    state->ap_first_minimal         = g_prefs.getBool("apmi",   false);
    state->nag_epas_faithful        = g_prefs.getBool("nagf",   false);
    state->soft_engage              = g_prefs.getBool("soft",   false);
    state->nag_burst                = g_prefs.getBool("nagb",   false);
    state->abort_guard              = g_prefs.getBool("abortg", false);
    state->suppress_speed_chime     = g_prefs.getBool("chime",  true);
    state->ignore_ota               = g_prefs.getBool("ignota", false);
    state->fsd_unlock               = g_prefs.getBool("unlock", false);
    state->force_fsd                = g_prefs.getBool("force",  false);
    state->china_mode               = g_prefs.getBool("china",  false);
    state->tlssc_restore            = g_prefs.getBool("tlssc",  false);
    state->precondition             = g_prefs.getBool("precond",false);
    state->emergency_vehicle_detect = g_prefs.getBool("emrg",   false);
    state->summon_unlock            = g_prefs.getBool("summon", false);
    state->continue_on_green        = g_prefs.getBool("cog",    false);
    state->assist_tlssc_bit38       = g_prefs.getBool("tlssc38", false);
    state->assist_rhd_override       = g_prefs.getBool("rhd",    false);
    state->assist_telemetry_off      = g_prefs.getBool("teloff", false);
    state->apmv3_branch             = g_prefs.getUChar("apmv3", 0xFF);  // AP branch/tier selector, 0xFF = OFF
    state->bms_output               = g_prefs.getBool("bms",    false);
    state->firmware_14x_warning     = g_prefs.getBool("14x",    true);
    state->blackbox_enabled         = g_prefs.getBool("bbx",    BLACKBOX_DEFAULT_ENABLED);
    state->track_mode_inject        = g_prefs.getBool("tmInj",  false);
    state->track_rotation_pct       = g_prefs.getUChar("tmRot", 100);
    state->track_stability_pct      = g_prefs.getUChar("tmStab", 30);
    state->track_post_cooling       = g_prefs.getBool("tmPC",   false);
    state->track_cmp_overclock      = g_prefs.getBool("tmCO",   false);
#if defined(BOARD_TTGO_DISPLAY)
    state->display_enabled          = g_prefs.getBool("disp",   true);
    state->display_brightness       = g_prefs.getUChar("disp_br", 50);
    state->display_timeout_s        = g_prefs.getUInt("disp_to",  60);
#endif
    state->sleep_idle_ms            = g_prefs.getUInt("sleep",  SLEEP_IDLE_MS);

    // WiFi
    if (g_prefs.isKey("wss")) g_prefs.getString("wss").toCharArray(state->wifi_ssid, sizeof(state->wifi_ssid));
    if (g_prefs.isKey("wsp")) g_prefs.getString("wsp").toCharArray(state->wifi_pass, sizeof(state->wifi_pass));
    state->wifi_hidden = g_prefs.getBool("wsh", false);
    if (g_prefs.isKey("stas")) g_prefs.getString("stas").toCharArray(state->wifi_sta_ssid, sizeof(state->wifi_sta_ssid));
    if (g_prefs.isKey("stap")) g_prefs.getString("stap").toCharArray(state->wifi_sta_pass, sizeof(state->wifi_sta_pass));

    state->op_mode = (OpMode)g_prefs.getUChar("mode", (uint8_t)OpMode_ListenOnly);
    // Manual HW selection (#110); TeslaHW_Unknown = auto-detect.
    state->hw_override = (TeslaHWVersion)g_prefs.getUChar("hwov", (uint8_t)TeslaHW_Unknown);
    state->fsd_protocol_mode = g_prefs.getUChar("fsdproto", 0);
    if (state->fsd_protocol_mode != 13 && state->fsd_protocol_mode != 14)
        state->fsd_protocol_mode = 0;

    // BSB CN HW3 driving style: 0=Auto, 1=Chill, 2=Normal, 3=Hurry.
    state->hw3_drive_style = g_prefs.getUChar("h3style", 0);
    if (state->hw3_drive_style > 5) state->hw3_drive_style = 0;

    // Source-matched smart offset settings
    state->hw3_offset_mode = g_prefs.getUChar("off_mod", 1);
    if (state->hw3_offset_mode > 2) state->hw3_offset_mode = 1;
    state->hw3_manual_offset = g_prefs.getUChar("man_off", 0);
    if (state->hw3_manual_offset > 63) state->hw3_manual_offset = 63;
    state->hw3_custom_pct[0] = g_prefs.getUChar("cp1", 30);
    state->hw3_custom_pct[1] = g_prefs.getUChar("cp2", 20);
    state->hw3_custom_pct[2] = g_prefs.getUChar("cp3", 10);
    state->hw3_custom_pct[3] = g_prefs.getUChar("cp4", 10);
    for (int i = 0; i < 4; ++i) if (state->hw3_custom_pct[i] > 63) state->hw3_custom_pct[i] = 63;

    // v1.4.33-style speed offset settings
    state->hw3_auto_speed = g_prefs.getBool("h3auto", false);
    state->hw3_custom_speed = g_prefs.getBool("h3cust", false);
    const uint8_t ct_def[5] = {45, 60, 75, 90, 105};
    const uint8_t hs_def[5] = {25, 25, 25, 25, 25};
    for (int i = 0; i < 5; ++i) {
        char k[8];
        snprintf(k, sizeof(k), "h3ct%d", i);
        state->hw3_custom_target[i] = g_prefs.getUChar(k, ct_def[i]);
        snprintf(k, sizeof(k), "h3hs%d", i);
        state->hw3_high_speed_pct[i] = g_prefs.getUChar(k, hs_def[i]);
    }
    state->hw3_offset_slew = g_prefs.getBool("h3slew", false);
    state->hw3_slew_rate = g_prefs.getUChar("h3rate", 5);
    if (state->hw3_slew_rate < 1 || state->hw3_slew_rate > 25) state->hw3_slew_rate = 5;
    state->hw3_high_speed_enable = g_prefs.getBool("h3high", false);
    state->hw4_offset = g_prefs.getUChar("h4off", 0);
    if (state->hw4_offset > 21) state->hw4_offset = 21;

    // Configurable nag-context signal mapping (#122)
    state->cfg_das_id        = g_prefs.getUShort("cdid",  0);
    state->cfg_apstate_byte  = g_prefs.getUChar("capb",   0);
    state->cfg_apstate_shift = g_prefs.getUChar("caps",   0);
    state->cfg_apstate_mask  = g_prefs.getUChar("capm",   0x0F);
    state->cfg_handson_byte  = g_prefs.getUChar("chob",   0);
    state->cfg_handson_shift = g_prefs.getUChar("chos",   0);
    state->cfg_handson_mask  = g_prefs.getUChar("chom",   0x0F);
    state->cfg_steer_id      = g_prefs.getUShort("csid",  0);
    state->cfg_steer_hi      = g_prefs.getUChar("cshi",   1);
    state->cfg_steer_lo      = g_prefs.getUChar("cslo",   0);

    Serial.printf("[NVS] Loaded: FSDUnlock=%d NAG=%d ContinuousAP=%d IgnoreOTA=%d China=%d Chime=%d Summon=%d COG=%d TLSSC38=%d RHD=%d TelOff=%d APMv3=%d TrkMode=%d/%u/%u/%d/%d Sleep=%u AP=\"%s\" STA=\"%s\" HIDDEN=%d\n",
                  state->fsd_unlock, state->nag_killer, state->continuous_ap, state->ignore_ota,
                  state->china_mode, state->suppress_speed_chime, state->summon_unlock,
                  state->continue_on_green, state->assist_tlssc_bit38, state->assist_rhd_override, state->assist_telemetry_off,
                  state->apmv3_branch, state->track_mode_inject, state->track_rotation_pct,
                  state->track_stability_pct, state->track_post_cooling, state->track_cmp_overclock,
                  state->sleep_idle_ms, state->wifi_ssid, state->wifi_sta_ssid,
                  state->wifi_hidden);
    g_prefs.end();
}

void prefs_clear() {
    g_prefs.begin(NS, /*readOnly=*/false);
    g_prefs.clear();
    g_prefs.end();
    Serial.println("[NVS] All settings erased — factory reset");
}

void prefs_save(const FSDState *state) {
    g_prefs.begin(NS, /*readOnly=*/false);
    g_prefs.putBool("ok",     true);
    g_prefs.putBool("nag",    state->nag_killer);
    g_prefs.putBool("contap", state->continuous_ap);
    g_prefs.putBool("apfirst",state->ap_first);
    g_prefs.putBool("apfe",   state->ap_first_edge);
    g_prefs.putBool("apmi",   state->ap_first_minimal);
    g_prefs.putBool("nagf",   state->nag_epas_faithful);
    g_prefs.putBool("soft",   state->soft_engage);
    g_prefs.putBool("nagb",   state->nag_burst);
    g_prefs.putBool("abortg", state->abort_guard);
    g_prefs.putBool("chime",  state->suppress_speed_chime);
    g_prefs.putBool("ignota", state->ignore_ota);
    g_prefs.putBool("unlock", state->fsd_unlock);
    g_prefs.putBool("force",  state->force_fsd);
    g_prefs.putBool("china",  state->china_mode);
    g_prefs.putBool("tlssc",  state->tlssc_restore);
    g_prefs.putBool("precond",state->precondition);
    g_prefs.putBool("emrg",   state->emergency_vehicle_detect);
    g_prefs.putBool("summon", state->summon_unlock);
    g_prefs.putBool("cog",    state->continue_on_green);
    g_prefs.putBool("tlssc38", state->assist_tlssc_bit38);
    g_prefs.putBool("rhd",    state->assist_rhd_override);
    g_prefs.putBool("teloff", state->assist_telemetry_off);
    g_prefs.putUChar("apmv3", state->apmv3_branch);  // AP branch/tier selector, 0xFF = OFF
    g_prefs.putBool("bms",    state->bms_output);
    g_prefs.putBool("14x",    state->firmware_14x_warning);
    g_prefs.putBool("bbx",    state->blackbox_enabled);
    g_prefs.putBool("tmInj",  state->track_mode_inject);
    g_prefs.putUChar("tmRot", state->track_rotation_pct);
    g_prefs.putUChar("tmStab",state->track_stability_pct);
    g_prefs.putBool("tmPC",   state->track_post_cooling);
    g_prefs.putBool("tmCO",   state->track_cmp_overclock);
#if defined(BOARD_TTGO_DISPLAY)
    g_prefs.putBool("disp",   state->display_enabled);
    g_prefs.putUChar("disp_br", state->display_brightness);
    g_prefs.putUInt("disp_to",  state->display_timeout_s);
#endif
    g_prefs.putUInt("sleep",  state->sleep_idle_ms);

    // WiFi
    g_prefs.putString("wss",  state->wifi_ssid);
    g_prefs.putString("wsp",  state->wifi_pass);
    g_prefs.putBool("wsh",    state->wifi_hidden);
    g_prefs.putString("stas", state->wifi_sta_ssid);
    g_prefs.putString("stap", state->wifi_sta_pass);

    g_prefs.putUChar("mode",  (uint8_t)state->op_mode);
    g_prefs.putUChar("hwov",  (uint8_t)state->hw_override);   // manual HW selection (#110)
    g_prefs.putUChar("fsdproto", state->fsd_protocol_mode);

    // BSB CN HW3 driving style
    g_prefs.putUChar("h3style", state->hw3_drive_style);

    // Source-matched smart offset settings
    g_prefs.putUChar("off_mod", state->hw3_offset_mode);
    g_prefs.putUChar("man_off", state->hw3_manual_offset);
    g_prefs.putUChar("cp1", state->hw3_custom_pct[0]);
    g_prefs.putUChar("cp2", state->hw3_custom_pct[1]);
    g_prefs.putUChar("cp3", state->hw3_custom_pct[2]);
    g_prefs.putUChar("cp4", state->hw3_custom_pct[3]);

    // v1.4.33-style speed offset settings
    g_prefs.putBool("h3auto", state->hw3_auto_speed);
    g_prefs.putBool("h3cust", state->hw3_custom_speed);
    for (int i = 0; i < 5; ++i) {
        char k[8];
        snprintf(k, sizeof(k), "h3ct%d", i);
        g_prefs.putUChar(k, state->hw3_custom_target[i]);
        snprintf(k, sizeof(k), "h3hs%d", i);
        g_prefs.putUChar(k, state->hw3_high_speed_pct[i]);
    }
    g_prefs.putBool("h3slew", state->hw3_offset_slew);
    g_prefs.putUChar("h3rate", state->hw3_slew_rate);
    g_prefs.putBool("h3high", state->hw3_high_speed_enable);
    g_prefs.putUChar("h4off", state->hw4_offset);

    // Configurable nag-context signal mapping (#122)
    g_prefs.putUShort("cdid", state->cfg_das_id);
    g_prefs.putUChar("capb",  state->cfg_apstate_byte);
    g_prefs.putUChar("caps",  state->cfg_apstate_shift);
    g_prefs.putUChar("capm",  state->cfg_apstate_mask);
    g_prefs.putUChar("chob",  state->cfg_handson_byte);
    g_prefs.putUChar("chos",  state->cfg_handson_shift);
    g_prefs.putUChar("chom",  state->cfg_handson_mask);
    g_prefs.putUShort("csid", state->cfg_steer_id);
    g_prefs.putUChar("cshi",  state->cfg_steer_hi);
    g_prefs.putUChar("cslo",  state->cfg_steer_lo);

    Serial.printf("[NVS] Saved: FSDUnlock=%d NAG=%d ContinuousAP=%d IgnoreOTA=%d China=%d Chime=%d Summon=%d COG=%d TLSSC38=%d RHD=%d TelOff=%d APMv3=%d TrkMode=%d/%u/%u/%d/%d Sleep=%u AP=\"%s\" STA=\"%s\" HIDDEN=%d\n",
                  state->fsd_unlock, state->nag_killer, state->continuous_ap, state->ignore_ota,
                  state->china_mode, state->suppress_speed_chime, state->summon_unlock,
                  state->continue_on_green, state->assist_tlssc_bit38, state->assist_rhd_override, state->assist_telemetry_off,
                  state->apmv3_branch, state->track_mode_inject, state->track_rotation_pct,
                  state->track_stability_pct, state->track_post_cooling, state->track_cmp_overclock,
                  state->sleep_idle_ms, state->wifi_ssid, state->wifi_sta_ssid,
                  state->wifi_hidden);
    g_prefs.end();
}
