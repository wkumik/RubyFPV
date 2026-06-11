# RF Auto Channel Selection — Masterplan & Handoff

Branch: `claude/rf-performance-channel-hopping-fd5oss`
Purpose: continuation/handoff document for finishing connect-time auto channel
selection. Read this first, then `docs/rf_auto_channel_selection.md` for the
original feasibility analysis and design rationale.

This is meant to be picked up on a machine with a Ruby Ground Station connected,
where the project can actually be built and tested.

---

## 1. Goal (recap)

Let the ground station automatically pick the cleanest channel at connect time,
based on a passive per-channel noise/occupancy scan. Targets interference and
congestion — NOT range/sensitivity. Frequency hopping (FHSS) is explicitly out
of scope for the wideband video link (see design doc §2).

---

## 2. Current state — what is DONE and committed

| Commit | Contents |
|---|---|
| `0b29e6f` | `docs/rf_auto_channel_selection.md` — feasibility + phased plan |
| `bf524a9` | `code/r_station/spectrum_scan.{h,cpp}` — scan engine; Makefile wiring |

### The scan engine (`code/r_station/spectrum_scan.{h,cpp}`)

Self-contained measurement core. Public API:

- `void spectrum_scan_default_config(type_spectrum_scan_config*)`
  Defaults: 350 ms dwell/channel, 2 sweeps, 60 ms settle after retune.
- `int spectrum_scan_run(int iInterfaceIndex, const u32* pFrequenciesKhz, int iCountFrequencies, const type_spectrum_scan_config*, type_spectrum_channel_result* pResults)`
  Retunes the interface per channel, dwells, accumulates per-channel:
  noise floor (best/worst dBm), strongest foreign signal, captured-frame count.
  Returns channels scanned, or -1.
- `int spectrum_scan_select_best(type_spectrum_channel_result*, int iCount)`
  Scores + ranks (lower = cleaner); returns index of best channel.

Result struct `type_spectrum_channel_result`: `uFrequencyKhz`, `iDbmNoiseFloor`,
`iDbmNoisePeak`, `iDbmSignalPeak`, `uFramesCaptured`, `uDwellMsTotal`, `iScore`,
`bHasData`. `SPECTRUM_DBM_INVALID` (1000) = no reading.

### KEY ARCHITECTURE DECISION (do not undo)

The station's normal RX pcap installs a **Ruby-only BPF filter**
(`code/radio/radiolink.c:670`: matches magic `0x13123456` + port byte). Foreign
WiFi traffic is dropped in-kernel and never reaches the existing RX path — so it
cannot measure occupancy or read noise on an empty channel.

Therefore the engine opens its **own permissive monitor-mode capture**
(`_spectrum_open_capture()`), no filter, so it sees ALL 802.11 frames on the
tuned channel. This is the whole reason the scanner works. Keep it.

### Caller contract (IMPORTANT)

Scanning retunes the interface away from any active link. The engine must only
be called when that interface is **idle** (pre-pairing / search / explicit
diagnostic). Caller is responsible for gating this. The engine saves and
restores the interface's original frequency around the sweep.

### Build status

NOT yet compiled — the dev container had no libpcap headers and this is a
cross-compiled project. All external symbols were verified against headers, but
**first real build happens on the GS machine.** Expect to fix minor issues
(see §5).

---

## 3. Remaining work (ordered)

### Step A — Validate the engine on hardware (do this FIRST)

Before any router/UI integration, prove the measurement core works on a real
adapter. Recommended: a tiny standalone test program (no router state needed).

1. Create `code/r_tests/test_spectrum_scan.cpp`:
   - `hardware_enumerate_radio_interfaces()` (or the project's init that
     populates `hardware_get_radio_info`) so interface 0 exists and is in
     monitor mode.
   - Build a frequency list with `getSupportedChannels(...)` from
     `code/base/config_radio.h`, or hardcode a handful (e.g. 2412000, 2437000,
     2462000, 5745000, 5805000 kHz).
   - Call `spectrum_scan_run(0, freqs, n, NULL, results)` then
     `spectrum_scan_select_best(results, n)`; print the table.
2. Add a `tests`-style target in the Makefile (mirror an existing `test_*`
   entry around line 164+ and the `MODULE_*` object lists) linking
   `$(FOLDER_STATION)/spectrum_scan.o` + the base/radio/common modules + pcap.
3. Run on the GS with an adapter in monitor mode. Sanity checks:
   - A known-busy 2.4 channel (next to home WiFi) shows higher `uFramesCaptured`.
   - A quiet 5.8 channel shows few/zero frames.
   - `iDbmNoiseFloor` is populated on at least some channels (if always
     `SPECTRUM_DBM_INVALID`/static, the Realtek driver isn't giving real noise —
     scoring already falls back to occupancy-only; confirm that path).
4. Tune `SPECTRUM_SCORE_W_*` weights and dwell/sweeps in `spectrum_scan.cpp` /
   `spectrum_scan_default_config()` against observed reality.

### Step B — Router integration (`code/r_station/ruby_rt_station.cpp`)

- Invoke the scan in the search/connect setup path, just before
  `radio_links_open_rxtx_radio_interfaces_for_search()` (see search-mode plumbing
  in `code/r_station/radio_links.cpp`, `radio_links_set_cards_frequencies_for_search()`).
- Gate behind a model/setting flag ("auto channel"). Only run when disconnected.
- Build the candidate list from the model's allowed channels / band whitelist.
- After the scan, apply the winner and continue pairing on it.

### Step C — Apply + persist the chosen channel

- Persist via the existing per-model frequency memory:
  `code/common/models_connect_frequencies.{h,cpp}` (already linked into
  `ruby_rt_station`).
- For a connected link, the existing `COMMAND_ID_SET_RADIO_LINK_FREQUENCY` path
  (`code/r_vehicle/ruby_rx_commands.cpp:~1944`) tells the vehicle to switch.
  At connect time prefer: station tunes to winner, vehicle is found there.

### Step D — Surface results to the UI (shared memory)

- The router (ruby_rt_station) runs the scan; `ruby_central` displays it.
  Add a small shared-memory block for `type_spectrum_channel_result[]` + a
  scan-state/progress field, or piggyback on the existing router→central
  message channel (see `code/r_central/process_router_messages.cpp`).

### Step E — UI (decided placement — see chat + design doc)

- **Manual scan tool** lives next to the MCS tester: "Optimize Links" /
  `MenuNegociateRadio` (`code/r_central/menu/menu_vehicle_radio.cpp:833`).
  Reuse `MenuNegociateRadio`'s progress-loop (`periodicLoop()`) and the
  results-table rendering pattern in `menu_vehicle_radio_rt_capab.cpp`.
- **Scan results** (per-channel noise/occupancy) annotate the rows in channel
  selection: `code/r_central/menu/menu_channels_select.cpp`.
- **Auto-channel toggle** (the user-facing feature) goes with channel selection
  / pairing. Start the manual scanner as an advanced/Developer-gated item like
  the MCS runtime-capabilities view; keep the auto toggle one-tap for users.

---

## 4. Build & test on the GS machine

```bash
# from repo root, on the Ground Station platform:
make station            # Pi default; builds ruby_rt_station incl. spectrum_scan.o
# or: make station RUBY_BUILD_ENV=radxa
# or: make station RUBY_BUILD_ENV=openipc

# if you add the standalone tester (Step A):
make tests              # then run ./test_spectrum_scan as root (monitor mode/pcap)
```

Notes:
- pcap: ensure libpcap dev headers are installed on the build host.
- Scanning needs an interface in monitor mode and root (raw capture + `iw set freq`).
- Running the scan drops any live link on that interface by design — test while
  disconnected.

---

## 5. Likely first-build gotchas to check

- libpcap: confirm `-lpcap` is in the station `_LDFLAGS` (it is for normal RX;
  the new file adds no new lib, just uses pcap already linked).
- Const-ness: `_spectrum_parse_frame()` C-casts a `const u8*` to
  `ieee80211_radiotap_header*` for the iterator — fine in C++, but check for a
  `-Werror`-promoted warning.
- `pcap_next_ex` data pointer type: `const u8*` vs `const u_char*` — should match
  (`u8` == `unsigned char`); adjust if the platform typedef differs.
- Two pcap handles on one monitor interface (engine's own + the router's): fine
  for capture on Linux, but if a driver misbehaves, fall back to temporarily
  swapping the existing handle's filter to permissive during the scan instead.
- Retune timing: if channels read as empty because the card hasn't settled,
  raise `uSettleMsAfterRetune` (currently 60 ms).

---

## 6. Open questions to resolve on hardware

1. Does the target Realtek driver report real per-channel `DBM_ANTNOISE`, or a
   static value? Determines whether noise scoring is used (engine auto-detects
   via `_spectrum_noise_is_usable()`, but confirm).
2. Best dwell vs. total-scan-time tradeoff for the field (currently 2×350 ms).
3. Whether to scan on one interface while others keep a link (diversity), or
   accept a full link drop during connect-time scan (simplest; current design).

---

## 7. Pointers (file:line)

- Scan engine: `code/r_station/spectrum_scan.cpp`, `...h`
- Ruby-only RX BPF filter (why we need our own capture): `code/radio/radiolink.c:670`
- Monitor capture setup to mirror: `code/radio/radiolink.c:553` (`_radio_open_interface_for_read_with_filter`)
- Radiotap noise/signal parsing reference: `code/radio/radiolink.c:1027-1163`
- Retune primitive: `radio_utils_set_interface_frequency()` — `code/base/utils.cpp:618` (decl `code/base/utils.h:64`)
- Search-mode channel set: `code/r_station/radio_links.cpp` (`radio_links_set_cards_frequencies_for_search`)
- Per-model frequency memory: `code/common/models_connect_frequencies.{h,cpp}`
- Vehicle freq-change command: `code/r_vehicle/ruby_rx_commands.cpp:~1944` (`COMMAND_ID_SET_RADIO_LINK_FREQUENCY`)
- MCS tester (UI sibling for scan tool): `code/r_central/menu/menu_negociate_radio.cpp`, results view `menu_vehicle_radio_rt_capab.cpp`
- Channel selection menu (results annotation target): `code/r_central/menu/menu_channels_select.cpp`
- Channel list accessors: `code/base/config_radio.h:65-83`
