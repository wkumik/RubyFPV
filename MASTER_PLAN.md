# RubyFPV × waybeam_venc — Integration Master Plan

Single source of truth for the OpenIPC/waybeam_venc integration into RubyFPV.

Status as of 2026-04-14.

---

## Phase 0 — Code abstraction (✅ done)

- [x] New `code/base/hardware_cam_backend.{h,cpp}` — runtime backend selector
- [x] Detection: `/etc/ruby_encoder` override → `/usr/bin/venc` probe → majestic fallback
- [x] Helpers: process name, binary path, config path, kill/reload commands, cli formatter, capability probes
- [x] Refactor `code/base/hardware_cam_maj.cpp` to route all hardcoded `"majestic"`, `killall -1/-9 majestic`, `/etc/majestic.yaml*`, `/usr/bin/majestic -s`, `cli -s`/`cli -d`, `ps -ae | grep majestic` through backend helpers
- [x] Per-backend field forks handled: `.isp.sensorConfig` ↔ `.isp.sensorBin`, `.outgoing.naluSize` ↔ `.outgoing.maxPayloadSize`, `.audio.srate` ↔ `.audio.sampleRate`, majestic-only fields skipped on waybeam (`.watchdog.*`, `.rtsp.*`, `.video1.*`, `.video0.sliceUnits`, `.isp.slowShutter`, `.fpv.enabled`)
- [x] New public API `hardware_camera_maj_request_idr()` — waybeam-only, hits `GET /request/idr`
- [x] Adapt callers: `video_source_majestic.cpp`, `video_sources.cpp`, `first_boot.cpp`, `r_start_vehicle.cpp`, `hardware_procs.cpp`
- [x] Wire `hardware_cam_backend.o` into Makefile (MODULE_BASE, ruby_start, ruby_rt_vehicle)
- [x] `LD_LIBRARY_PATH=/usr/lib/venc` in waybeam launch (no system lib overwrite)

## Phase 1 — Drop-in replacement (✅ proven on hardware)

- [x] Built `ruby_rt_vehicle` + `ruby_start` for OpenIPC SSC338Q (armhf glibc) in WSL via stock Ubuntu cross-toolchain
- [x] Validated waybeam standalone on RunCam WifiLink v2 (HTTP API, sensor unlock, ISP load, encoder threads)
- [x] Validated venc → majestic-stub → Ruby → GS (using only original Ruby binaries) — proves RTP compat
- [x] Validated full integrated path: patched Ruby auto-detects waybeam via `/etc/ruby_encoder`, spawns `/usr/bin/venc` with `LD_LIBRARY_PATH`, ingests RTP from venc, transmits via WFB, **video on GS confirmed**

## Phase 1.5 — GS-side cosmetic fixes (in progress)

- [ ] Fix "Video capture process malfunctioning" alarm firing once at boot (5s data-receive timeout too tight for waybeam startup)
- [ ] Fix bitrate not reported on GS (telemetry path may have majestic-specific assumption)
- [ ] Audit `r_central` / `r_station` for any majestic-specific code paths the controller-side still uses
- [ ] Make controller-side aware of which backend the vehicle is running (could come over telemetry as a string field)

## Phase 2 — Unix socket transport (planned)

- [ ] Switch `outgoing.server` from `udp://127.0.0.1:5600` to `unix://ruby_video` (abstract namespace)
- [ ] Add AF_UNIX SOCK_DGRAM recv path in `video_source_majestic.cpp` behind a transport abstraction; UDP stays as fallback
- [ ] Saves localhost UDP kernel overhead — pure latency win

## Phase 3 — Expose waybeam features in Ruby UI (planned)

- [ ] Wire `request_idr` into Ruby's "too many missing packets" recovery branch (faster post-blackout keyframe)
- [ ] Expose live resolution change in video menu (waybeam reinits in-process; majestic needs full restart)
- [ ] Expose ROI QP gradient (`fpv.roi*`) in video menu — center-priority encoding
- [ ] Expose gyro EIS toggle (`eis.*`, `imu.*`) — Star6E only
- [ ] Expose sensor FPS unlock — let FPS menu reach 120 on IMX415/335
- [ ] Probe `/api/v1/capabilities` once per connect, cache live vs restart-required fields, grey out restart-only controls in flight
- [ ] Optional debug menu for waybeam IQ tuning (60+ ISP params via `json_cli` passthrough)

## Phase 4 — Decided against (for now)

- ~~Wire waybeam's WFB-NG TX integration~~ — irrelevant, Ruby has its own radio stack
- ~~Maruko (i6c) backend~~ — Joakim says experimental; ship Star6E first
- ~~Gemini dual-VENC for stream + record~~ — possible Phase 5 once Phase 3 lands

## Phase 5 — Long-term ideas

- [ ] Gemini dual-VENC: simultaneous low-bitrate stream to goggles + 4K HQ recording to SD
- [ ] Audio path (waybeam audio key names differ from majestic — `.audio.sampleRate` vs `.audio.srate`)
- [ ] Update flow integration (`process_upload.cpp`) — Ruby's update mechanism currently expects majestic binary

---

## Known issues found during integration

### Ruby latent bugs (filed upstream)

- [x] [Issue #69](https://github.com/RubyFPV/RubyFPV/issues/69): `ruby_stop.sh` leaks `/dev/shm/sem.RUBY_STARTED_SEMAPHORE`, blocks subsequent `ruby_start`
- [x] [Issue #70](https://github.com/RubyFPV/RubyFPV/issues/70): Stale `version_ruby_base.txt` on device (says 11.1, repo says 11.8) — update flow doesn't replace it

### Build / deploy quirks

- [ ] `make all RUBY_BUILD_ENV=openipc` fails on Ubuntu 24.04 glibc 2.39 due to fortified `open()` requiring mode bits in `code/r_station/processor_rx_audio.cpp` — pre-existing source bug, unrelated to integration; workaround: `make vehicle` to skip station-side
- [ ] libpcap ABI mismatch: WSL Ubuntu 24.04 builds against libpcap 0.8 ABI, RunCam vehicle has libpcap 1.10.5; symlink workaround works but generates `radiotap_iterator_init < 0` softerrors. Proper fix: build against vehicle's libpcap headers OR static-link libpcap

### Hardware constraints (RunCam WifiLink v2 / SSC338Q)

- 5.7 MB total `/overlay` (persistent), ~1.5 MB free pre-deploy → strip binaries, ship only the 9 missing star6e libs (the other 3 are already on device)
- Vehicle has no `/boot` directory — override file path is `/etc/ruby_encoder` (not `/boot/encoder`)
- Vehicle has no `tcpdump` (BusyBox)
- Vehicle has no `sftp-server` — use `pscp -scp` or `rsync` over ssh

---

## Branches / artifacts

- Branch: [waybeam-integration-phase1](https://github.com/wkumik/RubyFPV/tree/waybeam-integration-phase1) on user fork
- Tip commit: [`6953e3c`](https://github.com/wkumik/RubyFPV/commit/6953e3c) — "Use LD_LIBRARY_PATH=/usr/lib/venc for waybeam launch"
- WSL build path: `/root/RubyFPV/` (Ubuntu 24.04, `arm-linux-gnueabihf-gcc 13.3.0`)
- Test vehicle: RunCam WifiLink v2 at `192.168.0.24` (root/12345)
