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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "../base/base.h"
#include "../base/config.h"
#include "../base/config_radio.h"
#include "../base/hardware.h"
#include "../base/hardware_radio.h"
#include "../base/hardware_procs.h"
#include "../radio/radiopackets2.h"
#include "../radio/radio_rx.h"
#include "shared_vars.h"
#include "timers.h"
#include "radio_links.h"
#include "rf_scan.h"

#define RF_SCAN_MAX_CHANNELS 128
#define RF_SCAN_DWELL_MS     280
#define RF_SCAN_RESULTS_FILE "/tmp/ruby_rf_scan_results.txt"

// Channel busy ratio in percent (0..100). Lower = cleaner channel.
// Written to the results file as the per-channel metric. A negative value
// means the radio card did not report usable survey data for that channel.
#define RF_SCAN_BUSY_INVALID (-1)

static bool  s_bRFScanInProgress = false;
static u32   s_uRFScanChannels[RF_SCAN_MAX_CHANNELS];
static int   s_iRFScanChannelsCount = 0;
static int   s_iRFScanCurrentChannel = 0;
static u32   s_uRFScanDwellStartTime = 0;
static FILE* s_pRFScanFile = NULL;
static u32   s_uRFScanBestFreqKhz = 0;
static int   s_iRFScanBestBusyPct = 0;

// Reads the channel busy ratio (in percent) for the currently-tuned channel
// from "iw dev <iface> survey dump". Returns RF_SCAN_BUSY_INVALID when the
// driver does not expose survey data (many monitor-mode RTL drivers do not),
// so the caller can surface "not supported" instead of fabricating a value.
static int _rf_scan_read_busy_pct(u32 uFreqKhz)
{
   char szIfaceName[64];
   szIfaceName[0] = 0;

   for ( int i = 0; i < hardware_get_radio_interfaces_count(); i++ )
   {
      radio_hw_info_t* pInfo = hardware_get_radio_info(i);
      if ( NULL == pInfo )
         continue;
      if ( ! hardware_radio_is_wifi_radio(pInfo) )
         continue;
      if ( ! pInfo->isConfigurable )
         continue;
      if ( 0 == hardware_radio_supports_frequency(pInfo, uFreqKhz) )
         continue;
      strncpy(szIfaceName, pInfo->szName, sizeof(szIfaceName) - 1);
      break;
   }

   if ( szIfaceName[0] == 0 )
      return RF_SCAN_BUSY_INVALID;

   char szCmd[256];
   char szOut[8192];
   szOut[0] = 0;
   snprintf(szCmd, sizeof(szCmd), "iw dev %s survey dump 2>/dev/null", szIfaceName);
   hw_execute_bash_command_silent(szCmd, szOut);

   int freq_mhz = (int)(uFreqKhz / 1000);
   char* pLine = szOut;
   bool bFoundFreq = false;
   long long llActive = -1;
   long long llBusy   = -1;

   while ( *pLine )
   {
      char* pNext = strchr(pLine, '\n');
      if ( pNext )
         *pNext = '\0';

      if ( !bFoundFreq )
      {
         int parsedMhz = 0;
         if ( sscanf(pLine, " frequency: %d MHz", &parsedMhz) == 1 && parsedMhz == freq_mhz )
            bFoundFreq = true;
      }
      else
      {
         // We are inside the survey block for the target frequency.
         long long llVal = 0;
         if ( sscanf(pLine, " channel active time: %lld ms", &llVal) == 1 )
            llActive = llVal;
         else if ( sscanf(pLine, " channel busy time: %lld ms", &llVal) == 1 )
            llBusy = llVal;
         // Reached the next survey block — stop.
         else if ( strstr(pLine, "frequency:") )
         {
            if ( pNext )
               *pNext = '\n';
            break;
         }
      }

      if ( pNext )
      {
         *pNext = '\n';
         pLine = pNext + 1;
      }
      else
         break;
   }

   if ( (llActive <= 0) || (llBusy < 0) )
      return RF_SCAN_BUSY_INVALID;

   long long llPct = (llBusy * 100) / llActive;
   if ( llPct < 0 )   llPct = 0;
   if ( llPct > 100 ) llPct = 100;
   return (int)llPct;
}

void rf_scan_start(u32 uBandFlags)
{
   if ( s_bRFScanInProgress )
      rf_scan_stop();

   s_iRFScanChannelsCount = 0;
   s_iRFScanCurrentChannel = 0;
   s_uRFScanDwellStartTime = 0;
   s_uRFScanBestFreqKhz   = 0;
   s_iRFScanBestBusyPct   = 0;

   u32* pChannels = NULL;
   int  nCount    = 0;

   if      ( uBandFlags == RADIO_HW_SUPPORTED_BAND_58  ) { pChannels = getChannels58();  nCount = getChannels58Count();  }
   else if ( uBandFlags == RADIO_HW_SUPPORTED_BAND_24  ) { pChannels = getChannels24();  nCount = getChannels24Count();  }
   else if ( uBandFlags == RADIO_HW_SUPPORTED_BAND_23  ) { pChannels = getChannels23();  nCount = getChannels23Count();  }
   else if ( uBandFlags == RADIO_HW_SUPPORTED_BAND_25  ) { pChannels = getChannels25();  nCount = getChannels25Count();  }
   else if ( uBandFlags == RADIO_HW_SUPPORTED_BAND_433 ) { pChannels = getChannels433(); nCount = getChannels433Count(); }
   else if ( uBandFlags == RADIO_HW_SUPPORTED_BAND_868 ) { pChannels = getChannels868(); nCount = getChannels868Count(); }
   else if ( uBandFlags == RADIO_HW_SUPPORTED_BAND_915 ) { pChannels = getChannels915(); nCount = getChannels915Count(); }

   if ( NULL == pChannels || nCount <= 0 )
   {
      log_softerror_and_alarm("RFScan: unknown or empty band flags %u, cannot scan.", uBandFlags);
      return;
   }

   if ( nCount > RF_SCAN_MAX_CHANNELS )
      nCount = RF_SCAN_MAX_CHANNELS;

   for ( int i = 0; i < nCount; i++ )
      s_uRFScanChannels[i] = pChannels[i];
   s_iRFScanChannelsCount = nCount;

   s_pRFScanFile = fopen(RF_SCAN_RESULTS_FILE, "w");
   if ( NULL == s_pRFScanFile )
   {
      log_softerror_and_alarm("RFScan: failed to open results file %s.", RF_SCAN_RESULTS_FILE);
      return;
   }

   s_bRFScanInProgress = true;
   log_line("RFScan: started scan for band flags %u, %d channels.", uBandFlags, s_iRFScanChannelsCount);
}

void rf_scan_periodic_loop()
{
   if ( !s_bRFScanInProgress )
      return;

   if ( s_iRFScanCurrentChannel >= s_iRFScanChannelsCount )
   {
      // All channels done
      if ( s_pRFScanFile )
      {
         fclose(s_pRFScanFile);
         s_pRFScanFile = NULL;
      }
      // Restore original frequencies
      if ( NULL != g_pCurrentModel )
         radio_links_set_cards_frequencies_and_params(-1);
      for ( int i = 0; i < hardware_get_radio_interfaces_count(); i++ )
         radio_rx_resume_interface(i);

      s_bRFScanInProgress = false;
      if ( s_uRFScanBestFreqKhz != 0 )
         log_line("RFScan: scan complete. Cleanest channel: %u kHz (%d%% busy).", s_uRFScanBestFreqKhz, s_iRFScanBestBusyPct);
      else
         log_line("RFScan: scan complete, but no usable survey data was reported by the radio card.");
      return;
   }

   u32 uFreqKhz = s_uRFScanChannels[s_iRFScanCurrentChannel];

   // Step 1: set frequency and start dwell
   if ( s_uRFScanDwellStartTime == 0 )
   {
      for ( int i = 0; i < hardware_get_radio_interfaces_count(); i++ )
         radio_rx_pause_interface(i, "RF scan channel change");
      radio_links_set_cards_frequencies_for_search(uFreqKhz, false, -1, -1, -1, -1);
      for ( int i = 0; i < hardware_get_radio_interfaces_count(); i++ )
         radio_rx_resume_interface(i);
      s_uRFScanDwellStartTime = g_TimeNow;
      return;
   }

   // Step 2: wait for dwell time
   if ( g_TimeNow < s_uRFScanDwellStartTime + RF_SCAN_DWELL_MS )
      return;

   // Step 3: read channel busy ratio and record
   int iBusyPct = _rf_scan_read_busy_pct(uFreqKhz);

   if ( s_pRFScanFile )
   {
      fprintf(s_pRFScanFile, "%u %d\n", uFreqKhz, iBusyPct);
      fflush(s_pRFScanFile);
   }

   if ( iBusyPct >= 0 )
   {
      if ( s_uRFScanBestFreqKhz == 0 || iBusyPct < s_iRFScanBestBusyPct )
      {
         s_iRFScanBestBusyPct = iBusyPct;
         s_uRFScanBestFreqKhz = uFreqKhz;
      }
   }

   log_line("RFScan: channel %d/%d  %u kHz  busy=%d%%.",
            s_iRFScanCurrentChannel + 1, s_iRFScanChannelsCount, uFreqKhz, iBusyPct);

   s_iRFScanCurrentChannel++;
   s_uRFScanDwellStartTime = 0;
}

void rf_scan_stop()
{
   if ( !s_bRFScanInProgress )
      return;

   if ( s_pRFScanFile )
   {
      fclose(s_pRFScanFile);
      s_pRFScanFile = NULL;
   }
   if ( NULL != g_pCurrentModel )
      radio_links_set_cards_frequencies_and_params(-1);
   for ( int i = 0; i < hardware_get_radio_interfaces_count(); i++ )
      radio_rx_resume_interface(i);

   s_bRFScanInProgress = false;
   log_line("RFScan: scan stopped.");
}

bool rf_scan_is_in_progress()
{
   return s_bRFScanInProgress;
}
