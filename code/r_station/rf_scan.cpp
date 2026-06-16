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
#include "../base/ruby_ipc.h"
#include "../radio/radiopackets2.h"
#include "../radio/radio_rx.h"
#include "shared_vars.h"
#include "timers.h"
#include "radio_links.h"
#include "rf_scan.h"

#define RF_SCAN_MAX_CHANNELS 128
#define RF_SCAN_DWELL_MS     280
#define RF_SCAN_RESULTS_FILE "/tmp/ruby_rf_scan_results.txt"

static bool  s_bRFScanInProgress = false;
static u32   s_uRFScanChannels[RF_SCAN_MAX_CHANNELS];
static int   s_iRFScanChannelsCount = 0;
static int   s_iRFScanCurrentChannel = 0;
static u32   s_uRFScanDwellStartTime = 0;
static int   s_iFDToCentral = -1;
static FILE* s_pRFScanFile = NULL;
static u32   s_uRFScanBestFreqKhz = 0;
static int   s_iRFScanBestNoise = 0;

static int _rf_scan_read_noise_dbm(u32 uFreqKhz)
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
      return -75;

   char szCmd[256];
   char szOut[8192];
   szOut[0] = 0;
   snprintf(szCmd, sizeof(szCmd), "iw dev %s survey dump 2>/dev/null", szIfaceName);
   hw_execute_bash_command_silent(szCmd, szOut);

   int freq_mhz = (int)(uFreqKhz / 1000);
   char* pLine = szOut;
   bool bFoundFreq = false;

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
         int noiseDbm = 0;
         if ( sscanf(pLine, " noise: %d dBm", &noiseDbm) == 1 )
         {
            if ( pNext )
               *pNext = '\n';
            return noiseDbm;
         }
         // Hit next survey block — stop
         if ( strstr(pLine, "frequency:") )
            break;
      }

      if ( pNext )
      {
         *pNext = '\n';
         pLine = pNext + 1;
      }
      else
         break;
   }
   return -75;
}

static void _rf_scan_send_result_to_central()
{
   if ( s_iFDToCentral < 0 )
      return;

   t_packet_header PH;
   radio_packet_init(&PH, PACKET_COMPONENT_LOCAL_CONTROL, PACKET_TYPE_LOCAL_CONTROLLER_RF_SCAN_RESULT, STREAM_ID_DATA);
   PH.vehicle_id_src  = 0;
   PH.vehicle_id_dest = s_uRFScanBestFreqKhz;
   PH.total_length    = sizeof(t_packet_header);
   radio_packet_compute_crc((u8*)&PH, sizeof(t_packet_header));
   ruby_ipc_channel_send_message(s_iFDToCentral, (u8*)&PH, sizeof(t_packet_header));
}

void rf_scan_start(u32 uBandFlags, int iFDToCentral)
{
   if ( s_bRFScanInProgress )
      rf_scan_stop();

   s_iFDToCentral         = iFDToCentral;
   s_iRFScanChannelsCount = 0;
   s_iRFScanCurrentChannel = 0;
   s_uRFScanDwellStartTime = 0;
   s_uRFScanBestFreqKhz   = 0;
   s_iRFScanBestNoise     = 0;

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
      log_line("RFScan: scan complete. Best channel: %u kHz (%d dBm).", s_uRFScanBestFreqKhz, s_iRFScanBestNoise);
      _rf_scan_send_result_to_central();
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

   // Step 3: read noise and record
   int iNoise = _rf_scan_read_noise_dbm(uFreqKhz);

   if ( s_pRFScanFile )
   {
      fprintf(s_pRFScanFile, "%u %d\n", uFreqKhz, iNoise);
      fflush(s_pRFScanFile);
   }

   if ( s_uRFScanBestFreqKhz == 0 || iNoise < s_iRFScanBestNoise )
   {
      s_iRFScanBestNoise   = iNoise;
      s_uRFScanBestFreqKhz = uFreqKhz;
   }

   log_line("RFScan: channel %d/%d  %u kHz  noise=%d dBm.",
            s_iRFScanCurrentChannel + 1, s_iRFScanChannelsCount, uFreqKhz, iNoise);

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

u32 rf_scan_get_best_freq_khz()
{
   return s_uRFScanBestFreqKhz;
}
