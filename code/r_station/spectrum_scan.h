/*
    Ruby Licence
    Copyright (c) 2020-2025 Petru Soroaga petrusoroaga@yahoo.com
    All rights reserved.

    Redistribution and/or use in source and/or binary forms, with or without
    modification, are permitted provided that the following conditions are met:
        * Redistributions and/or use of the source code (partially or complete) must retain
        the above copyright notice, this list of conditions and the following disclaimer
        in the documentation and/or other materials provided with the distribution.
        * Redistributions in binary form (partially or complete) must reproduce
        the above copyright notice, this list of conditions and the following disclaimer
        in the documentation and/or other materials provided with the distribution.
        * Copyright info and developer info must be preserved as is in the user
        interface, additions could be made to that info.
        * Neither the name of the organization nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.
        * Military use is not permitted.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
    ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE AUTHOR (PETRU SOROAGA) BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once
#include "../base/base.h"

// Passive RF spectrum scanner for connect-time auto channel selection.
//
// The scanner sweeps a list of candidate frequencies on a single radio
// interface and, for each one, retunes the interface and listens for a short
// dwell using a dedicated, *permissive* capture (no Ruby BPF filter), so that
// it sees all 802.11 traffic on the channel - not just Ruby frames. For each
// channel it accumulates:
//   - the radiotap noise floor (best/worst dBm reported by the driver),
//   - the strongest foreign signal seen,
//   - a frame count used as a channel-occupancy proxy.
//
// IMPORTANT (caller contract): scanning retunes the interface away from any
// active link, so it must only be run while that interface is NOT carrying a
// live link (e.g. before pairing / during search / explicit user-initiated
// diagnostic). The caller is responsible for that gating.
//
// Measurement caveats (by design, see docs/rf_auto_channel_selection.md):
//   - These chipsets do not export CCA/channel-busy time; frame count is the
//     best available occupancy proxy.
//   - Noise floor is only reported per captured frame; a perfectly quiet
//     channel yields no noise reading (which is itself a "clean" signal).
//   - Some Realtek builds report a coarse/static noise floor; scoring degrades
//     gracefully to occupancy + foreign-signal ranking when noise looks invalid.

#define MAX_SPECTRUM_SCAN_CHANNELS 64

// Sentinel used for "no dBm value captured" (real dBm readings are negative).
#define SPECTRUM_DBM_INVALID 1000

typedef struct
{
   u32 uFrequencyKhz;
   int iDbmNoiseFloor;   // best (lowest) noise floor seen, dBm; SPECTRUM_DBM_INVALID if none
   int iDbmNoisePeak;    // worst (highest) noise floor seen, dBm; SPECTRUM_DBM_INVALID if none
   int iDbmSignalPeak;   // strongest foreign signal seen, dBm; SPECTRUM_DBM_INVALID if none
   u32 uFramesCaptured;  // occupancy proxy: 802.11 frames seen during the dwell(s)
   u32 uDwellMsTotal;    // total dwell time actually spent on this channel
   int iScore;           // computed score, lower = cleaner; -1 if not scored yet
   int bHasData;         // 1 if any frame and/or noise reading was captured
} type_spectrum_channel_result;

typedef struct
{
   u32 uDwellMsPerChannel;  // listen time per channel per sweep
   int iSweeps;             // number of full passes over the channel list (>=1)
   u32 uSettleMsAfterRetune;// extra settle delay after retune before measuring
} type_spectrum_scan_config;

// Fill a config with sensible defaults (tune on real hardware).
void spectrum_scan_default_config(type_spectrum_scan_config* pConfig);

// Run a passive scan on the given radio interface across the listed
// frequencies. Opens its own permissive capture, retunes per channel, and
// accumulates per-channel results into pResults (caller-provided, sized for at
// least iCountFrequencies entries, up to MAX_SPECTRUM_SCAN_CHANNELS).
// Returns the number of channels scanned, or -1 on error.
int spectrum_scan_run(int iInterfaceIndex,
                      const u32* pFrequenciesKhz, int iCountFrequencies,
                      const type_spectrum_scan_config* pConfig,
                      type_spectrum_channel_result* pResults);

// Compute and store a score for every result, then return the index of the
// best (cleanest) channel, or -1 if none scorable. Lower score = cleaner.
int spectrum_scan_select_best(type_spectrum_channel_result* pResults, int iCount);
