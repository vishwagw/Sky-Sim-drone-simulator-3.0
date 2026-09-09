#!/usr/bin/env bash
# Build the SkySim physics core to WebAssembly. Requires Emscripten (emcc).
# Install: https://emscripten.org/docs/getting_started/downloads.html
set -e
cd "$(dirname "$0")/.."
mkdir -p web/public
emcc -std=c++20 -O3 -I include web/core/drone_core.cpp web/core/capi.cpp \
  -o web/public/skysim_core.js \
  -sMODULARIZE=1 -sEXPORT_NAME=SkySimCore \
  -sEXPORTED_RUNTIME_METHODS='["ccall","cwrap","HEAPF64"]' \
  -sEXPORTED_FUNCTIONS='["_skysim_create","_skysim_destroy","_skysim_reset","_skysim_arm","_skysim_set_attitude","_skysim_set_motors","_skysim_set_wind","_skysim_step","_skysim_get_obs","_skysim_altitude","_skysim_vspeed","_skysim_thrust","_skysim_roll","_skysim_pitch","_skysim_yaw","_skysim_power","_skysim_battery_voltage","_skysim_battery_soc","_skysim_battery_current","_skysim_battery_cutoff","_skysim_set_battery_thrust_coupling","_malloc","_free"]' \
  -sALLOW_MEMORY_GROWTH=1 -sENVIRONMENT=web
echo "built -> web/public/skysim_core.js + skysim_core.wasm"
