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

#include "spectrum_scan.h"
#include "../base/base.h"
#include "../base/hardware.h"
#include "../base/hardware_radio.h"
#include "../base/utils.h"
#include "../common/string_utils.h"
#include "../radio/radiotap.h"

#include <pcap.h>
#include <unistd.h>

// Scoring weights (lower total score = cleaner channel). Tunable on hardware.
#define SPECTRUM_SCORE_W_FRAMES   4   // per captured frame (occupancy proxy)
#define SPECTRUM_SCORE_W_SIGNAL   2   // per dBm of foreign signal above floor ref
#define SPECTRUM_SCORE_W_NOISE    3   // per dBm of noise floor above floor ref
#define SPECTRUM_DBM_FLOOR_REF  (-100) // reference floor; readings above this add cost

// Parse one captured radiotap frame, extracting the strongest signal and the
// quietest noise floor reported across antennas. Outputs use SPECTRUM_DBM_INVALID
// when the corresponding field is absent. Returns 1 on a successfully parsed
// radiotap header, 0 otherwise.
static int _spectrum_parse_frame(const u8* pData, int iLen, int* piSignalDbm, int* piNoiseDbm)
{
   *piSignalDbm = SPECTRUM_DBM_INVALID;
   *piNoiseDbm = SPECTRUM_DBM_INVALID;

   if ( (NULL == pData) || (iLen < (int)sizeof(struct ieee80211_radiotap_header)) )
      return 0;

   struct ieee80211_radiotap_iterator rti;
   if ( ieee80211_radiotap_iterator_init(&rti, (struct ieee80211_radiotap_header*)pData, iLen) < 0 )
      return 0;

   int iN = 0;
   while ( (iN = ieee80211_radiotap_iterator_next(&rti)) == 0 )
   {
      switch ( rti.this_arg_index )
      {
         case IEEE80211_RADIOTAP_DBM_ANTSIGNAL:
         {
            int iDbm = *((int8_t*)(rti.this_arg));
            if ( iDbm < 10 ) // sane received-signal reading
            {
               if ( (*piSignalDbm == SPECTRUM_DBM_INVALID) || (iDbm > *piSignalDbm) )
                  *piSignalDbm = iDbm; // strongest antenna
            }
            break;
         }
         case IEEE80211_RADIOTAP_DBM_ANTNOISE:
         {
            int iDbm = *((int8_t*)(rti.this_arg));
            if ( iDbm < 0 ) // sane noise-floor reading
            {
               if ( (*piNoiseDbm == SPECTRUM_DBM_INVALID) || (iDbm < *piNoiseDbm) )
                  *piNoiseDbm = iDbm; // quietest antenna = the floor
            }
            break;
         }
      }
   }
   return 1;
}

// Open a dedicated, permissive (no Ruby filter) monitor-mode capture on the
// interface so we observe all 802.11 traffic on the tuned channel. Returns the
// pcap handle or NULL on error.
static pcap_t* _spectrum_open_capture(radio_hw_info_t* pRadioHWInfo)
{
   char szErrbuf[PCAP_ERRBUF_SIZE];
   szErrbuf[0] = 0;

   pcap_t* pPcap = pcap_create(pRadioHWInfo->szName, szErrbuf);
   if ( NULL == pPcap )
   {
      log_softerror_and_alarm("[SpectrumScan] Unable to open capture on [%s]: %s", pRadioHWInfo->szName, szErrbuf);
      return NULL;
   }

   pcap_set_snaplen(pPcap, 256); // radiotap + 802.11 header is enough; we don't need payload
   pcap_set_promisc(pPcap, 1);
   pcap_set_timeout(pPcap, -1);
   pcap_set_immediate_mode(pPcap, 1);

   if ( pcap_activate(pPcap) != 0 )
   {
      log_softerror_and_alarm("[SpectrumScan] Failed to activate capture on [%s]: %s", pRadioHWInfo->szName, pcap_geterr(pPcap));
      pcap_close(pPcap);
      return NULL;
   }
   if ( pcap_setnonblock(pPcap, 1, szErrbuf) < 0 )
      log_softerror_and_alarm("[SpectrumScan] Failed to set nonblocking on [%s]: %s", pRadioHWInfo->szName, szErrbuf);

   int iLinkEncap = pcap_datalink(pPcap);
   if ( iLinkEncap != DLT_IEEE802_11_RADIO )
   {
      log_softerror_and_alarm("[SpectrumScan] Interface [%s] is not delivering radiotap headers (datalink %d); cannot scan.", pRadioHWInfo->szName, iLinkEncap);
      pcap_close(pPcap);
      return NULL;
   }
   // No filter installed on purpose: we want every frame on the channel.
   return pPcap;
}

// Drain any frames buffered from before/around the retune, without counting,
// for the given settle period.
static void _spectrum_drain_capture(pcap_t* pPcap, u32 uSettleMs)
{
   u32 uStart = get_current_timestamp_ms();
   struct pcap_pkthdr* pPktHeader = NULL;
   const u8* pPktData = NULL;
   while ( get_current_timestamp_ms() - uStart < uSettleMs )
   {
      int iRes = pcap_next_ex(pPcap, &pPktHeader, &pPktData);
      if ( iRes <= 0 )
         hardware_sleep_ms(1);
   }
}

// Listen on the currently tuned channel for uDwellMs, accumulating into pResult.
static void _spectrum_dwell(pcap_t* pPcap, u32 uDwellMs, type_spectrum_channel_result* pResult)
{
   u32 uStart = get_current_timestamp_ms();
   struct pcap_pkthdr* pPktHeader = NULL;
   const u8* pPktData = NULL;

   while ( get_current_timestamp_ms() - uStart < uDwellMs )
   {
      int iRes = pcap_next_ex(pPcap, &pPktHeader, &pPktData);
      if ( iRes <= 0 )
      {
         hardware_sleep_ms(1);
         continue;
      }

      int iSignalDbm = SPECTRUM_DBM_INVALID;
      int iNoiseDbm = SPECTRUM_DBM_INVALID;
      if ( ! _spectrum_parse_frame(pPktData, (int)pPktHeader->caplen, &iSignalDbm, &iNoiseDbm) )
         continue;

      pResult->uFramesCaptured++;
      pResult->bHasData = 1;

      if ( iSignalDbm != SPECTRUM_DBM_INVALID )
      {
         if ( (pResult->iDbmSignalPeak == SPECTRUM_DBM_INVALID) || (iSignalDbm > pResult->iDbmSignalPeak) )
            pResult->iDbmSignalPeak = iSignalDbm;
      }
      if ( iNoiseDbm != SPECTRUM_DBM_INVALID )
      {
         if ( (pResult->iDbmNoiseFloor == SPECTRUM_DBM_INVALID) || (iNoiseDbm < pResult->iDbmNoiseFloor) )
            pResult->iDbmNoiseFloor = iNoiseDbm;
         if ( (pResult->iDbmNoisePeak == SPECTRUM_DBM_INVALID) || (iNoiseDbm > pResult->iDbmNoisePeak) )
            pResult->iDbmNoisePeak = iNoiseDbm;
      }
   }
}

void spectrum_scan_default_config(type_spectrum_scan_config* pConfig)
{
   if ( NULL == pConfig )
      return;
   pConfig->uDwellMsPerChannel = 350;
   pConfig->iSweeps = 2;
   pConfig->uSettleMsAfterRetune = 60;
}

int spectrum_scan_run(int iInterfaceIndex,
                      const u32* pFrequenciesKhz, int iCountFrequencies,
                      const type_spectrum_scan_config* pConfig,
                      type_spectrum_channel_result* pResults)
{
   if ( (NULL == pFrequenciesKhz) || (NULL == pResults) || (iCountFrequencies <= 0) )
   {
      log_softerror_and_alarm("[SpectrumScan] Invalid parameters.");
      return -1;
   }
   if ( iCountFrequencies > MAX_SPECTRUM_SCAN_CHANNELS )
      iCountFrequencies = MAX_SPECTRUM_SCAN_CHANNELS;

   type_spectrum_scan_config localConfig;
   if ( NULL == pConfig )
   {
      spectrum_scan_default_config(&localConfig);
      pConfig = &localConfig;
   }

   radio_hw_info_t* pRadioHWInfo = hardware_get_radio_info(iInterfaceIndex);
   if ( NULL == pRadioHWInfo )
   {
      log_softerror_and_alarm("[SpectrumScan] No radio hardware info for interface %d.", iInterfaceIndex+1);
      return -1;
   }

   pcap_t* pPcap = _spectrum_open_capture(pRadioHWInfo);
   if ( NULL == pPcap )
      return -1;

   // Initialize results.
   for( int i=0; i<iCountFrequencies; i++ )
   {
      pResults[i].uFrequencyKhz = pFrequenciesKhz[i];
      pResults[i].iDbmNoiseFloor = SPECTRUM_DBM_INVALID;
      pResults[i].iDbmNoisePeak = SPECTRUM_DBM_INVALID;
      pResults[i].iDbmSignalPeak = SPECTRUM_DBM_INVALID;
      pResults[i].uFramesCaptured = 0;
      pResults[i].uDwellMsTotal = 0;
      pResults[i].iScore = -1;
      pResults[i].bHasData = 0;
   }

   int iSweeps = (pConfig->iSweeps > 0) ? pConfig->iSweeps : 1;

   log_line("[SpectrumScan] Starting scan on interface %d (%s): %d channels x %d sweep(s), %u ms dwell each.",
      iInterfaceIndex+1, pRadioHWInfo->szName, iCountFrequencies, iSweeps, pConfig->uDwellMsPerChannel);

   u32 uOriginalFreq = pRadioHWInfo->uCurrentFrequencyKhz;

   for( int iSweep=0; iSweep<iSweeps; iSweep++ )
   for( int i=0; i<iCountFrequencies; i++ )
   {
      if ( ! radio_utils_set_interface_frequency(NULL, iInterfaceIndex, -1, pFrequenciesKhz[i], NULL, 0) )
      {
         log_softerror_and_alarm("[SpectrumScan] Failed to tune interface %d to %s; skipping channel.",
            iInterfaceIndex+1, str_format_frequency(pFrequenciesKhz[i]));
         continue;
      }

      _spectrum_drain_capture(pPcap, pConfig->uSettleMsAfterRetune);
      _spectrum_dwell(pPcap, pConfig->uDwellMsPerChannel, &pResults[i]);
      pResults[i].uDwellMsTotal += pConfig->uDwellMsPerChannel;
   }

   // Restore the interface to where it started, so the caller resumes cleanly.
   if ( uOriginalFreq != 0 )
      radio_utils_set_interface_frequency(NULL, iInterfaceIndex, -1, uOriginalFreq, NULL, 0);

   pcap_close(pPcap);

   log_line("[SpectrumScan] Scan complete on interface %d.", iInterfaceIndex+1);
   for( int i=0; i<iCountFrequencies; i++ )
   {
      log_line("[SpectrumScan] %s: frames=%u, noiseFloor=%d dBm, noisePeak=%d dBm, signalPeak=%d dBm%s",
         str_format_frequency(pResults[i].uFrequencyKhz),
         pResults[i].uFramesCaptured,
         pResults[i].iDbmNoiseFloor, pResults[i].iDbmNoisePeak, pResults[i].iDbmSignalPeak,
         pResults[i].bHasData ? "" : " (no data)");
   }

   return iCountFrequencies;
}

// Returns 1 if the noise readings across the scan look usable (not all invalid
// and not suspiciously static, which some Realtek drivers report).
static int _spectrum_noise_is_usable(const type_spectrum_channel_result* pResults, int iCount)
{
   int iValid = 0;
   int iMin = SPECTRUM_DBM_INVALID;
   int iMax = SPECTRUM_DBM_INVALID;
   for( int i=0; i<iCount; i++ )
   {
      if ( pResults[i].iDbmNoiseFloor == SPECTRUM_DBM_INVALID )
         continue;
      iValid++;
      if ( (iMin == SPECTRUM_DBM_INVALID) || (pResults[i].iDbmNoiseFloor < iMin) )
         iMin = pResults[i].iDbmNoiseFloor;
      if ( (iMax == SPECTRUM_DBM_INVALID) || (pResults[i].iDbmNoiseFloor > iMax) )
         iMax = pResults[i].iDbmNoiseFloor;
   }
   if ( iValid < 2 )
      return 0;
   // If every channel reports the same floor, the driver isn't giving us real
   // per-channel noise; fall back to occupancy-only scoring.
   if ( iMax - iMin < 2 )
      return 0;
   return 1;
}

int spectrum_scan_select_best(type_spectrum_channel_result* pResults, int iCount)
{
   if ( (NULL == pResults) || (iCount <= 0) )
      return -1;

   int bUseNoise = _spectrum_noise_is_usable(pResults, iCount);

   int iBestIndex = -1;
   int iBestScore = 0;
   for( int i=0; i<iCount; i++ )
   {
      int iScore = 0;

      // Occupancy dominates: real foreign traffic is the strongest interference
      // indicator we can actually measure on these chipsets.
      iScore += (int)pResults[i].uFramesCaptured * SPECTRUM_SCORE_W_FRAMES;

      // Penalize a strong foreign signal present on the channel.
      if ( pResults[i].iDbmSignalPeak != SPECTRUM_DBM_INVALID )
      {
         int iAboveRef = pResults[i].iDbmSignalPeak - SPECTRUM_DBM_FLOOR_REF;
         if ( iAboveRef > 0 )
            iScore += iAboveRef * SPECTRUM_SCORE_W_SIGNAL;
      }

      // Penalize a high noise floor, only when the driver gives usable numbers.
      if ( bUseNoise && (pResults[i].iDbmNoiseFloor != SPECTRUM_DBM_INVALID) )
      {
         int iAboveRef = pResults[i].iDbmNoiseFloor - SPECTRUM_DBM_FLOOR_REF;
         if ( iAboveRef > 0 )
            iScore += iAboveRef * SPECTRUM_SCORE_W_NOISE;
      }

      pResults[i].iScore = iScore;

      if ( (iBestIndex < 0) || (iScore < iBestScore) )
      {
         iBestIndex = i;
         iBestScore = iScore;
      }
   }

   if ( iBestIndex >= 0 )
      log_line("[SpectrumScan] Selected cleanest channel: %s (score %d, noise scoring %s).",
         str_format_frequency(pResults[iBestIndex].uFrequencyKhz), iBestScore,
         bUseNoise ? "on" : "off");

   return iBestIndex;
}
