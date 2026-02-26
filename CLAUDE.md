# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

RubyFPV is an open-source digital radio system for UAVs, drones, planes, and remote vehicles. It provides real-time video, audio, telemetry, and remote control over multiple redundant radio links (433/868/915 MHz and 2.4/5.8 GHz). Written primarily in C/C++ (~750 source/header files), it targets embedded Linux platforms: Raspberry Pi, Radxa, and OpenIPC.

## Build Commands

```bash
# Build everything (default target is Raspberry Pi)
make all

# Build for a specific platform
make all RUBY_BUILD_ENV=radxa
make all RUBY_BUILD_ENV=openipc

# Build subsystems
make vehicle        # Vehicle-side binaries (ruby_start, ruby_utils, ruby_tx_telemetry, ruby_rt_vehicle)
make station        # Ground station binaries (ruby_controller, ruby_rt_station, ruby_tx_rc, ruby_rx_telemetry)
make ruby_central   # Central UI application
make ruby_plugins   # OSD plugins (shared libraries)
make tests          # Test binaries (test_gpio, test_port_rx, test_port_tx, test_link)

# Build individual test
make test_link
make test_port_rx

# Clean
make clean
make cleanstation   # Clean station-side only

# Radxa convenience script
./make_radxa.sh
```

Platform is selected via `RUBY_BUILD_ENV` env var which sets preprocessor defines: `RUBY_BUILD_HW_PLATFORM_PI`, `RUBY_BUILD_HW_PLATFORM_RADXA`, or `RUBY_BUILD_HW_PLATFORM_OPENIPC`.

## Architecture

### Multi-Process Design

The system runs as multiple cooperating processes communicating via shared memory and IPC:

**Vehicle side:**
- `ruby_start` — Bootstrap/initialization, hardware detection, first boot setup
- `ruby_rt_vehicle` — Main runtime: video capture, radio TX, packet processing
- `ruby_tx_telemetry` — Reads flight controller telemetry (MAVLink/MSP/LTM) and transmits
- `ruby_rx_commands` — Receives commands from ground station
- `ruby_rx_rc` — Receives RC input

**Controller/ground station side:**
- `ruby_controller` — Main controller process
- `ruby_rt_station` — Runtime: radio RX, video reception/output, relay handling
- `ruby_central` — Full UI application (menus, OSD, rendering)
- `ruby_rx_telemetry` — Receives and processes telemetry
- `ruby_tx_rc` — Transmits RC commands to vehicle

**Shared utilities:** `ruby_logger`, `ruby_alive` (watchdog), `ruby_video_proc`, `ruby_update`, `ruby_dbg`

### Source Directory Layout

| Directory | Purpose |
|-----------|---------|
| `code/base/` | Core: hardware abstraction, config, GPIO, I2C, encryption, shared memory, IPC |
| `code/radio/` | Radio protocol: packet formats, FEC, TX/RX, radiotap, duplicate detection |
| `code/common/` | Shared utilities: radio stats, string utils, relay utils |
| `code/r_vehicle/` | Vehicle runtime: video sources, telemetry, adaptive video, radio links |
| `code/r_station/` | Station runtime: video RX/recording, radio links, RC transmission |
| `code/r_central/` | Central UI app with subdirs: `menu/` (60+ menu screens), `osd/` (HUD), `oled/` |
| `code/renderer/` | Graphics backends: Cairo+DRM (Radxa), raw framebuffer+dispmanx (Pi) |
| `code/r_start/` | System startup, radio init, first boot |
| `code/r_utils/` | Utility binaries (logger, DHCP, SIK config, update worker, etc.) |
| `code/r_tests/` | Test programs for GPIO, ports, links, audio, joystick, etc. |
| `code/r_player/` | Radxa video player (Rockchip MPP + DRM) |
| `code/r_plugins_osd/` | OSD plugins compiled as `.so` shared libraries |
| `code/public/` | Public SDK headers for plugin development |
| `code/utils/` | Controller/vehicle utility functions |
| `mavlink/` | MAVLink protocol XML definitions |
| `res/` | Fonts, icons, images, camera calibration files |

### Key Architectural Patterns

**Platform abstraction:** Conditional compilation via `RUBY_BUILD_HW_PLATFORM_*` defines. Each platform has different graphics (OpenVG/EGL on Pi, Cairo/DRM on Radxa), GPIO (wiringPi vs libgpiod), and video (MMAL vs Rockchip MPP) backends.

**Radio protocol:** Custom packet format with 1250-byte max payload, 1500-byte max total. Supports 8 radio streams, 4 video streams. Uses Forward Error Correction (FEC). Stream IDs: 0=data, 1=telemetry, 2=audio, 3=data2, 4=video_1.

**IPC:** Shared memory segments (e.g., `/SSMRVideo` 512KB buffer) for inter-process data. Message queue logging (queue ID 123). `ruby_ipc` for process communication.

**Plugin system:** OSD plugins are `.so` shared libraries implementing `init()`, `getName()`, `getUID()`, `render()`. Core plugins support custom video/data/telemetry streams and hardware access.

**Telemetry parsing:** Supports MAVLink (multiple dialects: ArduPilot, INAV, etc.), MSP (MultiWii), and LTM (Lightweight Telemetry) protocols.

### Makefile Module System

The Makefile defines reusable object file groups:
- `MODULE_BASE` / `MODULE_BASE2` — Core base objects
- `MODULE_COMMON` — Shared utilities
- `MODULE_RADIO` — Radio protocol objects
- `MODULE_MODELS` — Vehicle model management
- `MODULE_VEHICLE` / `MODULE_STATION` — Runtime-specific objects
- `CENTRAL_MENU_ALL1..6`, `CENTRAL_OSD_ALL`, etc. — UI component groups

Compilation flags: `-Wall -O2 -fdata-sections -ffunction-sections` with `-Wl,--gc-sections` for dead code elimination.

## Platform-Specific Dependencies

- **Pi:** wiringPi, Broadcom MMAL/VideoCore, OpenVG, EGL, freetype, libpng, libjpeg
- **Radxa:** libdrm, Cairo, SDL2, libgpiod, libi2c, Rockchip MPP
- **OpenIPC:** Minimal (librt, libpcap, pthread only)
- **All platforms:** librt, libpcap, pthread
