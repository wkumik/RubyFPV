# Plan — Media Player Fixes + DJI-style Thumbnail Browser

Branch: `feat/media-player-thumbnails`
Status: **PLAN (not yet implemented)** — 2026-06-12
Target board: Radxa Zero 3W GS (`RUBY_BUILD_ENV=radxa`), player = `ruby_player_radxa`

Two requests from the user:
1. **Playback plays sped up** (videos run faster than real time).
2. **Add thumbnails + filename labels** to the media browser, "like DJI does it".

---

## Part 1 — Sped-up playback

### Root cause (read from source, not yet hardware-confirmed)

The **offline file-playback path** in `code/r_player/ruby_player_radxa.cpp`
(`_thread_play_file`, the loop around lines 208–280) is the problem. It is
entirely separate from the live RX path (which decodes waybeam HEVC correctly —
proven on hardware at ~15% CPU). Three compounding bugs, all HEVC-specific:

1. **No PTS / hand-rolled pacing.** Frames are paced by a fixed sleep:
   ```c
   // ruby_player_radxa.cpp:247
   long int miliSecs = (1000/(g_iFileFPS-4));   // the "-4" is a hack comment:
   // "OpenIPC generates faster FPS as IFrames are not part of computation"
   ```
   The container's real presentation timestamps are ignored. `g_iFileFPS`
   comes from the GS `.info` sidecar (`video_playback.cpp:85`) and **defaults
   to 30** (`ruby_player_radxa.cpp:91, 1003-1004`).

2. **Frame boundaries detected with an H.264-only NAL parser** even for HEVC:
   ```c
   // ruby_player_radxa.cpp:225
   uNALType = (*pTmp) & 0x1F;                 // H.264 = 5-bit type, IDR == 5
   // ruby_player_radxa.cpp:241
   if ( ! parser_h264_is_signaling_nal(uNALType) )
   ```
   HEVC NAL type is `(byte >> 1) & 0x3F` (6-bit). With H.264 masking, HEVC NAL
   headers decode to bogus types, so slice/frame boundary counting
   (`g_iFileDetectedSlices`, `g_iFileTempSlices`) is wrong → the pacing sleep
   fires on the wrong cadence → wrong (faster) playback rate.

3. **Codec chosen by filename, not content.** `g_bUseH265Decoder` is set true
   **only** if the filename contains `.h265` (`ruby_player_radxa.cpp:1013`).
   - waybeam onboard recordings are `**.ts**` (MPEG-TS HEVC) → no `.h265` in the
     name → MPP is initialized as an **H.264** decoder for an HEVC stream.
   - The `.ts` MPEG-TS container is also **not demuxed**: the file is read raw
     and scanned for Annex-B start codes (`0x00000001`) then fed straight to
     MPP. TS has 188-byte packet framing, so this is wrong for `.ts` regardless
     of codec.

Net: any HEVC recording (and especially `.ts` onboard recordings) plays at the
wrong rate / wrong codec. GS-side H.265 recordings hit bugs #1 and #2; onboard
`.ts` recordings hit all three.

### Fix options (recommended: B, with A as the correctness backstop)

**Option A — Honor real timing via the container (most correct).**
Demux with the platform's MPP demuxer or libavformat and feed access units with
their PTS, sleeping to match wall clock. Heaviest change; needs a demux
dependency in the player.

**Option B — ffprobe-driven metadata + correct HEVC frame detection
(recommended, smallest correct change).** The GS already has `ffmpeg`/`ffprobe`
(`/usr/bin/`). On play:
  1. `ffprobe` the file for **codec** (`hevc`/`h264`), **avg_frame_rate**, and
     **duration**. Drive `g_bUseH265Decoder` and `g_iFileFPS` from this — kill
     the filename-extension sniff and the `-fps` default-30 / `-4` hack.
  2. Add an **HEVC NAL-type path** in the pacing loop:
     `uNALType = (byte >> 1) & 0x3F`; treat VPS(32)/SPS(33)/PPS(34)/AUD(35)/
     SEI(39,40) as signaling; IDR is types 19/20. Pick the parser by the
     detected codec.
  3. For real PTS accuracy, prefer pacing off the decoder's output cadence
     rather than a fixed per-NAL sleep (removes the `-4` fudge entirely).

**Option C — Remux/normalize on import (pairs well with B for `.ts`).**
For `.ts` files (onboard recordings), remux losslessly to a clean elementary
stream or `.mp4` before playback (`ffmpeg -i x.ts -c copy …`) and probe fps from
the source. Avoids in-player TS demux. Could be done once at "copy from drone"
time and cached next to the file.

### Verification (no-celebration A/B)
- Play a known-duration onboard clip (we have `rec_00h08m41s_e646.ts`, ffprobe =
  HEVC 1280×720 **59 fps, 10.82 s**). Wall-clock the playback before/after:
  pre-fix runs short (sped up); post-fix must take ~10.8 s.
- Test both an HEVC `.ts` (onboard) and a GS-side recording.

---

## Part 2 — DJI-style thumbnail browser

### Current state
`code/r_central/menu/menu_storage.cpp` (`MenuStorage`) is a **text menu list**:
files enumerated via `readdir` (lines ~472, ~512), each row drawn with
`drawText` and a "Play" affordance. No images.

### Design
A thumbnail grid with a filename label under each tile, navigable with the
existing menu input. The render engine (`g_pRenderEngine`, Cairo on Radxa)
already draws images (used by OSD plugins / `oled_icon_loader`), so tiles are
feasible without new graphics deps.

**Thumbnail generation (GS has ffmpeg):**
- Lazily, on first browse, generate `<name>.thumb.jpg` if missing:
  `ffmpeg -ss 0 -i <file> -frames:v 1 -vf scale=320:-1 <name>.thumb.jpg`
  (use `-ss` after a keyframe; for `.ts` HEVC this works directly).
- Cache the `.thumb.jpg` next to the recording so it is generated once.
- Generation must be **async / off the UI thread** (spawn a worker, show a
  placeholder tile until ready) so the menu never blocks on ffmpeg.
- Alternative/longer-term: have waybeam emit a thumbnail at record-stop on the
  drone (cheaper, no GS decode), pulled alongside the clip.

**Browser UI:**
- New grid menu (e.g. `MenuMediaGallery`) or a grid mode in `MenuStorage`:
  N columns of tiles (thumbnail + filename + duration/size), pageable with the
  existing Prev/Next-page items, selection highlight, [OK] = play.
- Reuse the duration/fps/size metadata from the ffprobe step in Part 1 (show it
  on the tile, DJI-style).

### Open questions to confirm with the user
- Grid size / tile dimensions for the GS screen resolution.
- Thumbnail at first frame vs. mid-clip (DJI uses first frame).
- Should onboard `.ts` files on the drone SD be browsable/playable directly from
  the GS, or only after "copy to GS"? (Affects whether we demux `.ts` in-player
  or remux on import.)

---

## Sequencing
1. **Part 1 first** (correctness) — it is small, high-value, and the fps/codec
   probing it adds is reused by Part 2's tile metadata.
2. **Part 2** on top, starting with lazy ffmpeg thumbnail generation + a grid
   render, then polish (async generation, metadata overlay).

## Git / base note
This planning branch was cut at the current `feat/onboard-sd-recording` HEAD
(`32fc208`), which includes the **abandoned AP-unpack patch**. The media-player
changes touch only `ruby_player_radxa.cpp`, `mpp_core.cpp`, `video_playback.cpp`,
and `menu_storage.cpp` — none of which the AP patch modified — so the eventual
implementation branch should be **rebased/cut from a clean base** (`71aea0d` or
upstream) excluding `32fc208`. Keep `.o` build artifacts out of the diff (known
repo-hygiene issue).

---

## Part 3 — OTA download of onboard recordings + remux to .mp4

**Motivation (user, 2026-06-12):** waybeam onboard recording writes HEVC `.ts`
to the drone SD (`/mnt/mmcblk0p1/ruby/`). The user wants to pull those clips to
the GS over the air and have them playable/shareable as `.mp4`, browsable in the
same DJI-style thumbnail grid as Part 2.

### What already exists
- GS↔vehicle **segmented file transfer** is already implemented:
  `COMMAND_ID_DOWNLOAD_FILE` / `COMMAND_ID_DOWNLOAD_FILE_SEGMENT` in
  `r_central/handle_commands.cpp` (file-id based, segment ack/retry loop). This
  is the transport to reuse — do NOT invent a new one.
- Onboard recordings land in a known dir on the drone SD with a matching `.osd`
  telemetry sidecar (written by `rx_osd_recording_vehicle.*`, shipped in the
  waybeam-integration PR #78).

### Gaps to build
1. **List recordings (new command).** A `COMMAND_ID_LIST_ONBOARD_RECORDINGS`
   (vehicle → GS) returning {filename, size, duration, has-osd} for each `.ts`
   under the recordings dir. Without this the GS has no file-id to download.
2. **Map recording → downloadable file-id.** Wire the listed files into the
   existing `s_*FileToDownload*` machinery so the segmented transfer can fetch
   an arbitrary recording by path, not just the current hardcoded targets.
3. **GS-side import UI.** In the recordings/media browser: per-clip
   "Download from drone" action → progress (reuse the existing
   "Downloading NN%" popup) → file saved into the GS media folder so Part 2's
   thumbnail grid picks it up.
4. **Remux .ts → .mp4 (container only, no transcode).** waybeam writes HEVC in
   an MPEG-TS container; `.ts → .mp4` is a pure remux:
   `ffmpeg -i in.ts -c copy out.mp4`. **Verify ffmpeg is present on the Radxa
   GS** (`which ffmpeg`); if not, either ship it or keep `.ts` and rely on the
   player demuxing TS directly (Part 1 already needs HEVC playback). Remux on
   import, keep the `.osd` sidecar alongside the `.mp4`.

### Sequencing / dependency
- Depends on **Part 1** (HEVC playback) being done first — no point importing
  HEVC clips the player can't play.
- The list/download commands are vehicle-side, so this part needs a coordinated
  GS+vehicle build (see [[feedback_match_gs_drone_protocol]]).

### Open questions
- Download over the FPV link competes with live video bandwidth — gate downloads
  to when disarmed / not streaming, or throttle? (DJI does it on the ground.)
- Delete-after-download option, or leave clips on the drone SD?
