#pragma once

/* ---- Color ---- */
#define LV_COLOR_DEPTH 16
// This board's esp_lcd_panel_draw_bitmap() path expects big-endian RGB565
// (matches Waveshare's own official lv_conf.h for this exact board) -
// different transport than the main tiltdash project's manual QSPI driver,
// which does its own byte-order handling instead and needs this at 0.
#define LV_COLOR_16_SWAP 1

/* ---- Memory / perf ---- */
#define LV_MEM_CUSTOM 0
#define LV_USE_LOG 0

/* ---- Widgets ---- */
#define LV_USE_LABEL 1

/* ---- Fonts ---- */
#define LV_FONT_MONTSERRAT_14 1
#define LV_FONT_MONTSERRAT_20 1
#define LV_FONT_MONTSERRAT_22 1
#define LV_FONT_MONTSERRAT_40 1
#define LV_FONT_MONTSERRAT_48 1
#define LV_FONT_DEFAULT &lv_font_montserrat_22

#define LV_USE_CANVAS 1
#define LV_USE_BTN 1
#define LV_USE_LABEL 1
