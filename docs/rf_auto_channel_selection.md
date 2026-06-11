# RF Auto Channel Selection — Feasibility & Implementation Plan

Status: design proposal (not yet implemented)
Scope: connect-time automatic channel selection driven by a passive per-channel
noise/SNR scan. Frequency *hopping* (FHSS) is explicitly out of scope — see
"Why not hopping" below.

---

## 1. Goal

Let the ground station pick the cleanest available channel automatically instead
of forcing the pilot to guess a frequency. This targets the single most common
real-world RF problem: **interference / congestion** (a field full of other
pilots, 2.4 GHz WiFi clutter). It does **not** improve range, sensitivity, or
multipath — those are antenna/diversity/datarate concerns and are partly handled
already by adaptive video and datarate negotiation.

Two phases are proposed:

- **Phase 1 — connect-time selection** (recommended first step): scan candidate
  channels before the link goes live, pick the best, then pair on it.
- **Phase 2 — in-flight adaptive switching** (later, higher risk): migrate to a
  better channel mid-flight when the active one degrades. Deferred; the
  coordination problem (commanding a switch over a link that is already failing)
  makes this materially riskier and it can make things worse on a false trigger.

---

## 2. Why not frequency hopping (FHSS)

True per-packet hopping like ELRS/Crossfire control links is **not** a good fit
for the RubyFPV video link and is intentionally excluded:

- The video link is **wideband and sustained-throughput** at MCS rates. Control
  links hop cheaply because they are narrowband and low-rate.
- The supported WiFi chipsets (RTL8812AU/EU class) retune in **milliseconds**,
  not microseconds. Hopping fast enough to gain interference immunity would
  destroy video throughput and create a hard TX/RX synchronization problem on
  both ends.

Hopping could in principle make sense on a *separate narrowband control/telemetry
link*, but that is a niche and not part of this proposal.

---

## 3. Chipset scanning feasibility — verdict: FEASIBLE, no driver work

The key question for any auto-selection feature is: **can we measure per-channel
noise/interference on these chipsets, in monitor mode, with what we already
have?** The answer is yes, via passive radiotap noise extraction during a dwell.
We do **not** need `iw survey dump`, netlink work, or driver patches.

### 3.1 What already exists in the codebase

| Capability | Status | Evidence |
|---|---|---|
| Retune an interface to any supported freq | Implemented & widely used | `radio_utils_set_interface_frequency()` — `code/base/utils.cpp:618` (shells out to `iw dev <if> set freq` on Pi, `iwconfig ... freq` elsewhere) |
| Sequential channel sweep + dwell loop | Proven in pairing search | `radio_links_set_cards_frequencies_for_search()` — `code/r_station/radio_links.cpp` (search mode retunes all interfaces to one freq and dwells) |
| Per-packet **noise floor** from radiotap | Implemented | `IEEE80211_RADIOTAP_DBM_ANTNOISE` parsed at `code/radio/radiolink.c:1040`, aggregated to `iDbmNoiseLast/Min/Max` at `radiolink.c:1138-1163` |
| Per-packet **signal** + derived **SNR** | Implemented | `IEEE80211_RADIOTAP_DBM_ANTSIGNAL` at `radiolink.c:1027`; SNR = signal − noise at `radiolink.c:1106` |
| Per-interface signal/noise aggregates in shared mem | Implemented | `type_runtime_radio_rx_signal_info` (signalInfoAll / Video / Data) — `code/base/hardware_radio.h`, mirrored in `code/base/shared_mem_radio.h` |
| All three chipset families emit radiotap in monitor mode | Yes | Realtek (RTL8812AU/EU, RTL8733BU), Atheros (ath9k_htc), Ralink (RT2800USB, 2.4 GHz only) |

### 3.2 What is missing (the actual work)

- **No per-frequency bucketing.** Signal/noise is aggregated *per interface,
  current frequency only* (`uCurrentFrequencyKhz`). When the card retunes, the
  old aggregates are overwritten — there is no per-channel history.
- **No scan orchestration** that loops a channel list, dwells, snapshots stats,
  and ranks results.
- **No `iw scan` / `iw survey dump` usage anywhere** — and we should not add it.
  Monitor mode generally blocks a normal scan, and survey support across the
  Realtek out-of-tree drivers is unreliable. Passive radiotap is the robust path.

### 3.3 Important measurement caveats (must be designed around)

These determine how much to trust the result; the plan accounts for them:

1. **Noise floor with no traffic ≠ interference under load.** `DBM_ANTNOISE` is
   the driver's noise estimate; on a quiet channel it may read a static floor and
   under-report bursty interference (e.g. another video TX keying up
   intermittently). Mitigation: dwell long enough to catch bursts, and combine
   noise floor with **received-frame density** (count of foreign frames seen
   during dwell) as an occupancy proxy.
2. **No CCA / channel-busy-time.** These drivers do not export airtime/CCA via
   radiotap, so we cannot get a true "% busy" figure without driver work.
   Foreign-frame count during the dwell is our best available proxy.
3. **Realtek noise quality varies.** Some Realtek builds report a coarse or
   fixed antnoise. The algorithm must degrade gracefully: if antnoise looks
   invalid/static across all channels, fall back to ranking by foreign-frame
   density + signal of detected foreign traffic alone.
4. **Dwell vs. total scan time.** ~50–200 ms per retune (incl. error recovery)
   plus dwell. A 6–12 channel shortlist at ~300–500 ms dwell ⇒ ~3–8 s total,
   acceptable at connect time, not acceptable mid-flight (Phase 2 concern).

---

## 4. Phase 1 — connect-time auto channel selection

### 4.1 User-facing behavior

1. Pilot enables "Auto channel" for a band (or a curated channel whitelist) in
   the pairing / radio-link menu.
2. On connect/search start, the station scans the whitelist, builds a per-channel
   score, picks the best, and pairs there.
3. The chosen frequency is persisted per-model (existing
   `models_connect_frequencies` mechanism) so the next connection starts there.

Note: the **vehicle** must end up on the same channel. At connect time the vehicle
is in its search/default state, so the natural integration point is: station
scans → station tunes to the winning channel → normal pairing proceeds (vehicle is
found there, or commanded via the existing `COMMAND_ID_SET_RADIO_LINK_FREQUENCY`
once linked). This avoids the Phase-2 coordination paradox entirely because the
scan happens *before* there is a link to protect.

### 4.2 Data model

Add a per-frequency stats bucket (new struct, kept separate from the per-interface
"current" aggregates so we don't disturb the hot RX path semantics):

```c
// new: code/base/shared_mem_radio.h (or a dedicated scan header)
typedef struct {
   u32 uFrequencyKhz;
   int iDbmNoiseMin, iDbmNoiseMax, iDbmNoiseAvg;   // floor over the dwell
   int iDbmSignalMax;                              // strongest foreign signal
   int iSNRWorst;                                  // worst-case headroom
   u32 uForeignFrameCount;                         // occupancy proxy
   u32 uDwellMs;
   u32 uLastScanTime;
} type_channel_scan_result;
```

### 4.3 Scan loop (new)

Add to `code/r_station/radio_links.cpp`, reusing the existing retune primitive:

```c
// pseudocode
bool radio_links_scan_channels(const u32* pFreqListKhz, int iCount,
                               u32 uDwellMs, type_channel_scan_result* pOut)
{
   for (int c = 0; c < iCount; c++) {
      // 1. retune every interface that supports this freq
      radio_utils_set_interface_frequency(... pFreqListKhz[c] ...);
      // 2. reset per-channel accumulators
      // 3. dwell: pump RX (reuse the existing capture path) for uDwellMs,
      //    accumulating DBM_ANTNOISE / DBM_ANTSIGNAL and counting foreign frames
      // 4. snapshot into pOut[c]
   }
   return true;
}
```

Two viable ways to feed accumulators during the dwell:

- **Preferred / low-touch:** add a small hook in the existing radiotap
  aggregation block (`radiolink.c:1138-1163`) that, when a global
  "scan in progress" flag is set, also pushes the same per-packet noise/signal
  into the current channel's bucket. Minimal change to a proven code path.
- **Alternative:** sample the existing per-interface `iDbmNoiseLast/Min/Max`
  immediately after the dwell. Simpler, but coarser (loses the foreign-frame
  count). Use only if the hook is deemed too invasive.

### 4.4 Scoring / selection

Rank channels (lower score = better). Starting heuristic, to be tuned on real
hardware:

```
score(ch) = w_noise * normalized(iDbmNoiseAvg)        // quieter is better
          + w_occ   * normalized(uForeignFrameCount)  // emptier is better
          + w_burst * normalized(iDbmNoiseMax)         // penalize bursty noise
```

- If antnoise is invalid/static across the board (caveat 3.3.3), drop `w_noise`
  to 0 and rank on occupancy + foreign signal strength only.
- Prefer keeping the previously-good channel if its score is within a hysteresis
  margin of the new best (avoid churn when differences are noise).
- Respect the user's whitelist and per-interface band support
  (`hardware_radioindex_supports_frequency`).

### 4.5 Integration point

`code/r_station/ruby_rt_station.cpp` — invoke the scan in search/connect setup,
just before opening interfaces for search
(`radio_links_open_rxtx_radio_interfaces_for_search()`), gated behind the
"auto channel" model flag. Persist the winner via the existing
`models_connect_frequencies` path.

### 4.6 UI

`code/r_central/menu/` (alongside `menu_channels_select.cpp`): an "Auto channel"
toggle + whitelist editor, and ideally a results view showing the per-channel
noise/occupancy map (this doubles as a poor-man's spectrum display, useful on its
own).

### 4.7 Files to touch (Phase 1)

| File | Change |
|---|---|
| `code/base/shared_mem_radio.h` | add `type_channel_scan_result`, scan-in-progress flag |
| `code/radio/radiolink.c` | optional hook (~ line 1138) to bucket noise/signal per channel during scan |
| `code/r_station/radio_links.cpp` | new `radio_links_scan_channels()` loop |
| `code/r_station/ruby_rt_station.cpp` | call scan at connect; apply + persist winner |
| `code/common/radio_stats.c` | per-channel reset/update helpers (optional) |
| `code/r_central/menu/*` | auto-channel toggle, whitelist, results view |

Rough size: ~300–500 LOC, no kernel/driver changes.

---

## 5. Phase 2 — in-flight adaptive switching (deferred)

Sketch only; recommend deferring until Phase 1 is validated.

- **Trigger:** sustained SNR/quality drop on the active channel (reuse the link-
  loss detection that already drives adaptive video).
- **Coordination paradox:** the command to switch must traverse the link that is
  already degrading. Robust approach is a **pre-agreed fallback channel
  sequence** both ends walk in lockstep on a link-loss timeout (like a blind
  rendezvous), rather than an explicit commanded hop.
- **Risk:** false triggers can drop a working link or desync the two ends. Needs
  conservative thresholds, hysteresis, and a guaranteed return-to-home channel.
- **Cost:** background scanning competes with live video for airtime/tuning, so
  it must be opportunistic (e.g. brief dwell on a spare diversity interface, if
  present) rather than retuning the primary link.

---

## 6. Recommendation

Build **Phase 1** first. It delivers the real-world interference win, reuses
infrastructure that already exists and is proven (retune + radiotap noise
aggregation + search-mode sweep + per-model frequency persistence), needs no
driver work, and sidesteps the coordination risk that makes in-flight switching
dangerous. Validate the noise/occupancy scoring on real RTL8812AU/EU and
ath9k_htc hardware before considering Phase 2.
