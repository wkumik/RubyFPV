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

#include "menu.h"
#include "menu_rf_scan.h"
#include "menu_item_select.h"
#include "../../base/config_radio.h"
#include "../../base/hardware_radio.h"
#include "../../common/string_utils.h"

static MenuRFScan* s_pLastRFScanMenu = NULL;

MenuRFScan::MenuRFScan(int iVehicleRadioLink, u32 uFreqKhz)
:Menu(MENU_ID_RF_SCAN, "RF Spectrum Scan", NULL)
{
   m_Width  = 0.92f;
   m_xPos   = 0.04f;
   m_yPos   = 0.04f;

   m_iVehicleRadioLink = iVehicleRadioLink;
   m_uCurrentFreqKhz   = uFreqKhz;
   m_uBandFlags        = getBand(uFreqKhz);

   m_iScanChannelsCount  = 0;
   m_iChannelsScanned    = 0;
   m_uBestFreqKhz        = 0;
   m_iBestChannelIndex   = -1;
   m_bScanInProgress     = false;
   m_bScanComplete       = false;

   memset(m_iNoiseDbm, 0, sizeof(m_iNoiseDbm));

   // Build channel list for this band
   u32* pChannels = NULL;
   int  nCount    = 0;
   if      ( m_uBandFlags == RADIO_HW_SUPPORTED_BAND_58  ) { pChannels = getChannels58();  nCount = getChannels58Count();  }
   else if ( m_uBandFlags == RADIO_HW_SUPPORTED_BAND_24  ) { pChannels = getChannels24();  nCount = getChannels24Count();  }
   else if ( m_uBandFlags == RADIO_HW_SUPPORTED_BAND_23  ) { pChannels = getChannels23();  nCount = getChannels23Count();  }
   else if ( m_uBandFlags == RADIO_HW_SUPPORTED_BAND_25  ) { pChannels = getChannels25();  nCount = getChannels25Count();  }
   else if ( m_uBandFlags == RADIO_HW_SUPPORTED_BAND_433 ) { pChannels = getChannels433(); nCount = getChannels433Count(); }
   else if ( m_uBandFlags == RADIO_HW_SUPPORTED_BAND_868 ) { pChannels = getChannels868(); nCount = getChannels868Count(); }
   else if ( m_uBandFlags == RADIO_HW_SUPPORTED_BAND_915 ) { pChannels = getChannels915(); nCount = getChannels915Count(); }

   if ( pChannels && nCount > 0 )
   {
      if ( nCount > RF_SCAN_MAX_CHANNELS )
         nCount = RF_SCAN_MAX_CHANNELS;
      for ( int i = 0; i < nCount; i++ )
      {
         m_uScanChannels[i] = pChannels[i];
         m_iNoiseDbm[i]     = -999;
      }
      m_iScanChannelsCount = nCount;
   }

   // Reserve space for the spectrum chart at the bottom
   float height_text = g_pRenderEngine->textHeight(g_idFontMenu);
   addExtraHeightAtEnd(height_text * 12.0f);

   m_IndexRescan         = addMenuItem(new MenuItem("Start Scan", "Scan all channels in this band and measure noise floor."));
   m_IndexUseRecommended = addMenuItem(new MenuItem("Use Recommended Channel", "Apply the cleanest channel to this radio link."));
   m_IndexCancel         = addMenuItem(new MenuItem("Cancel", "Close without changing the frequency."));

   enableMenuItem(m_IndexUseRecommended, false);

   char szTitle[128];
   const char* szBand = "Unknown";
   if      ( m_uBandFlags == RADIO_HW_SUPPORTED_BAND_58  ) szBand = "5.8 GHz";
   else if ( m_uBandFlags == RADIO_HW_SUPPORTED_BAND_24  ) szBand = "2.4 GHz";
   else if ( m_uBandFlags == RADIO_HW_SUPPORTED_BAND_23  ) szBand = "2.3 GHz";
   else if ( m_uBandFlags == RADIO_HW_SUPPORTED_BAND_25  ) szBand = "2.5 GHz";
   else if ( m_uBandFlags == RADIO_HW_SUPPORTED_BAND_433 ) szBand = "433 MHz";
   else if ( m_uBandFlags == RADIO_HW_SUPPORTED_BAND_868 ) szBand = "868 MHz";
   else if ( m_uBandFlags == RADIO_HW_SUPPORTED_BAND_915 ) szBand = "915 MHz";
   snprintf(szTitle, sizeof(szTitle), "RF Spectrum Scan — %s — %d channels", szBand, m_iScanChannelsCount);
   setTitle(szTitle);

   s_pLastRFScanMenu = this;
}

MenuRFScan::~MenuRFScan()
{
   if ( s_pLastRFScanMenu == this )
      s_pLastRFScanMenu = NULL;
}

void MenuRFScan::onShow()
{
   Menu::onShow();
   _startScan();
}

void MenuRFScan::_startScan()
{
   for ( int i = 0; i < m_iScanChannelsCount; i++ )
      m_iNoiseDbm[i] = -999;
   m_iChannelsScanned  = 0;
   m_uBestFreqKhz      = 0;
   m_iBestChannelIndex = -1;
   m_bScanInProgress   = true;
   m_bScanComplete     = false;

   enableMenuItem(m_IndexUseRecommended, false);
   m_pMenuItems[m_IndexUseRecommended]->setTitle("Use Recommended Channel");

   send_control_message_to_router(PACKET_TYPE_LOCAL_CONTROLLER_RF_SCAN_START, (u32)m_uBandFlags);
   log_line("MenuRFScan: sent scan start for band flags %d, %d channels.", m_uBandFlags, m_iScanChannelsCount);
}

bool MenuRFScan::periodicLoop()
{
   if ( m_bScanInProgress )
      _readResultsFile();
   return false;
}

void MenuRFScan::_readResultsFile()
{
   FILE* f = fopen(RF_SCAN_RESULTS_FILE, "r");
   if ( NULL == f )
      return;

   u32 uFreqKhz = 0;
   int iNoise   = 0;
   while ( fscanf(f, "%u %d", &uFreqKhz, &iNoise) == 2 )
   {
      for ( int i = 0; i < m_iScanChannelsCount; i++ )
      {
         if ( m_uScanChannels[i] == uFreqKhz )
         {
            m_iNoiseDbm[i] = iNoise;
            break;
         }
      }
   }
   fclose(f);

   m_iChannelsScanned = 0;
   for ( int i = 0; i < m_iScanChannelsCount; i++ )
      if ( m_iNoiseDbm[i] != -999 )
         m_iChannelsScanned++;

   if ( m_iChannelsScanned >= m_iScanChannelsCount && m_iScanChannelsCount > 0 )
   {
      m_bScanInProgress = false;
      m_bScanComplete   = true;

      int bestNoise = 0;
      for ( int i = 0; i < m_iScanChannelsCount; i++ )
      {
         if ( m_iNoiseDbm[i] == -999 )
            continue;
         if ( m_iBestChannelIndex < 0 || m_iNoiseDbm[i] < bestNoise )
         {
            bestNoise           = m_iNoiseDbm[i];
            m_iBestChannelIndex = i;
            m_uBestFreqKhz      = m_uScanChannels[i];
         }
      }

      if ( m_iBestChannelIndex >= 0 )
      {
         enableMenuItem(m_IndexUseRecommended, true);
         char szLabel[128];
         snprintf(szLabel, sizeof(szLabel), "Use Recommended: %s", str_format_frequency(m_uBestFreqKhz));
         m_pMenuItems[m_IndexUseRecommended]->setTitle(szLabel);
      }
   }
}

void MenuRFScan::Render()
{
   RenderPrepare();
   float yTop = RenderFrameAndTitle();
   float y    = yTop;
   for ( int i = 0; i < m_ItemsCount; i++ )
      y += RenderItem(i, y);

   float height_text_s = g_pRenderEngine->textHeight(g_idFontMenuSmall);

   // Status line
   char szStatus[256];
   if ( m_bScanInProgress )
      snprintf(szStatus, sizeof(szStatus), "Scanning...  %d / %d channels", m_iChannelsScanned, m_iScanChannelsCount);
   else if ( m_bScanComplete )
      snprintf(szStatus, sizeof(szStatus), "Scan complete  |  %d channels  |  Cleanest: %s",
               m_iScanChannelsCount, m_uBestFreqKhz ? str_format_frequency(m_uBestFreqKhz) : "N/A");
   else
      snprintf(szStatus, sizeof(szStatus), "Press Start Scan to begin  |  %d channels in band", m_iScanChannelsCount);

   g_pRenderEngine->setColors(get_Color_MenuText());
   g_pRenderEngine->drawText(m_RenderXPos + m_sfMenuPaddingX, y, g_idFontMenuSmall, szStatus);
   y += height_text_s * 1.5f;

   // Spectrum chart
   float fChartH = m_RenderYPos + m_RenderTotalHeight - y - m_sfMenuPaddingY;
   if ( fChartH > 0.05f )
      _renderSpectrumChart(m_RenderXPos + m_sfMenuPaddingX, y,
                           m_RenderWidth - 2.0f * m_sfMenuPaddingX, fChartH);

   RenderEnd(yTop);
}

void MenuRFScan::_renderSpectrumChart(float xPos, float yPos, float fWidth, float fHeight)
{
   if ( m_iScanChannelsCount == 0 )
      return;

   float wPx = g_pRenderEngine->getPixelWidth();
   float hPx = g_pRenderEngine->getPixelHeight();
   float hTs = g_pRenderEngine->textHeight(g_idFontMenuSmall);

   // Reserve left margin for Y-axis labels and bottom for freq labels
   float fLabelW = g_pRenderEngine->textWidth(g_idFontMenuSmall, "-100");
   float fGrX    = xPos + fLabelW + wPx * 4;
   float fGrW    = fWidth - fLabelW - wPx * 4;
   float fGrY    = yPos;
   float fGrH    = fHeight - hTs * 1.6f;
   float fGrB    = fGrY + fGrH;

   // Y range: -50 (top) to -100 (bottom)  — lower = quieter = better
   const float kNoiseTop = -50.0f;
   const float kNoiseBt  = -100.0f;

   // Grid lines + Y labels
   int yTicks[] = { -50, -60, -70, -80, -90, -100 };
   for ( int t = 0; t < 6; t++ )
   {
      float dbm  = (float)yTicks[t];
      float frac = (dbm - kNoiseTop) / (kNoiseBt - kNoiseTop);
      float gy   = fGrY + frac * fGrH;

      g_pRenderEngine->setStroke(0.22f, 0.25f, 0.35f, 1.0f);
      g_pRenderEngine->setStrokeSize(1);
      g_pRenderEngine->drawLine(fGrX, gy, fGrX + fGrW, gy);

      char szLbl[16];
      snprintf(szLbl, sizeof(szLbl), "%d", yTicks[t]);
      g_pRenderEngine->setColors(get_Color_MenuItemDisabledText());
      g_pRenderEngine->drawTextLeft(fGrX - wPx * 3, gy - hTs * 0.5f, g_idFontMenuSmall, szLbl);
   }

   // Axes
   g_pRenderEngine->setStroke(0.35f, 0.38f, 0.52f, 1.0f);
   g_pRenderEngine->setStrokeSize(1);
   g_pRenderEngine->drawLine(fGrX, fGrY, fGrX, fGrB);
   g_pRenderEngine->drawLine(fGrX, fGrB, fGrX + fGrW, fGrB);

   // Bars
   float barW = fGrW / (float)m_iScanChannelsCount;
   float gap  = barW * 0.10f;
   if ( gap < wPx ) gap = wPx;

   for ( int i = 0; i < m_iScanChannelsCount; i++ )
   {
      float bx0 = fGrX + i * barW + gap;
      float bx1 = fGrX + (i + 1) * barW - gap;
      float bw  = bx1 - bx0;

      if ( m_iNoiseDbm[i] == -999 )
      {
         // Not yet scanned: dim outline
         g_pRenderEngine->setFill(0, 0, 0, 0);
         g_pRenderEngine->setStroke(0.18f, 0.20f, 0.28f, 0.6f);
         g_pRenderEngine->setStrokeSize(1);
         g_pRenderEngine->drawRect(bx0, fGrY, bw, fGrH);
         continue;
      }

      float frac   = ((float)m_iNoiseDbm[i] - kNoiseTop) / (kNoiseBt - kNoiseTop);
      if ( frac < 0.0f ) frac = 0.0f;
      if ( frac > 1.0f ) frac = 1.0f;
      float barTop = fGrY + frac * fGrH;

      // Colour by noise level (spectrum style: shorter/lower bar = cleaner)
      float cr, cg, cb;
      if      ( m_iNoiseDbm[i] <= -88 ) { cr = 0.20f; cg = 0.78f; cb = 0.38f; }  // green
      else if ( m_iNoiseDbm[i] <= -76 ) { cr = 0.94f; cg = 0.75f; cb = 0.18f; }  // yellow
      else                               { cr = 0.85f; cg = 0.25f; cb = 0.22f; }  // red

      // Highlight best channel
      if ( i == m_iBestChannelIndex )
      {
         g_pRenderEngine->setFill(0.10f, 0.32f, 0.16f, 0.35f);
         g_pRenderEngine->setStroke(0.18f, 0.86f, 0.46f, 1.0f);
         g_pRenderEngine->setStrokeSize(2);
         g_pRenderEngine->drawRect(bx0 - wPx * 2, fGrY, bw + wPx * 4, fGrH);
         cr = 0.18f; cg = 0.86f; cb = 0.46f;
      }

      g_pRenderEngine->setFill(cr, cg, cb, 1.0f);
      g_pRenderEngine->setStrokeSize(0);
      g_pRenderEngine->drawRect(bx0, barTop, bw, fGrB - barTop);

      // Freq label for every 4th channel, first, last, and best
      bool bLabel = ( i == 0 ) || ( i == m_iScanChannelsCount - 1 ) ||
                    ( i == m_iBestChannelIndex ) || ( i % 4 == 0 );
      if ( bLabel )
      {
         char szF[16];
         snprintf(szF, sizeof(szF), "%u", m_uScanChannels[i] / 1000);
         float lw  = g_pRenderEngine->textWidth(g_idFontMenuSmall, szF);
         float cx  = bx0 + bw * 0.5f;
         g_pRenderEngine->setColors(( i == m_iBestChannelIndex ) ? get_Color_IconSucces() : get_Color_MenuItemDisabledText());
         g_pRenderEngine->drawText(cx - lw * 0.5f, fGrB + hPx * 4, g_idFontMenuSmall, szF);
      }

      // Annotation bubble for recommended channel
      if ( i == m_iBestChannelIndex && m_bScanComplete )
      {
         char szLn1[32], szLn2[32];
         snprintf(szLn1, sizeof(szLn1), "%u MHz", m_uScanChannels[i] / 1000);
         snprintf(szLn2, sizeof(szLn2), "%d dBm", m_iNoiseDbm[i]);

         float tw1  = g_pRenderEngine->textWidth(g_idFontMenu, szLn1);
         float tw2  = g_pRenderEngine->textWidth(g_idFontMenuSmall, szLn2);
         float bubW = (tw1 > tw2 ? tw1 : tw2) + wPx * 16;
         float bubH = hTs + g_pRenderEngine->textHeight(g_idFontMenu) + hPx * 10;
         float cx   = bx0 + bw * 0.5f;
         float bubX = cx - bubW * 0.5f;
         if ( bubX < fGrX ) bubX = fGrX;
         if ( bubX + bubW > fGrX + fGrW ) bubX = fGrX + fGrW - bubW;
         float bubY = barTop - bubH - hPx * 14;
         if ( bubY < fGrY + hPx * 2 ) bubY = fGrY + hPx * 2;

         // Arrow (simple V shape)
         g_pRenderEngine->setStroke(0.18f, 0.86f, 0.46f, 1.0f);
         g_pRenderEngine->setStrokeSize(2);
         float aY = barTop - hPx * 2;
         float aW = wPx * 7;
         float aH = hPx * 10;
         g_pRenderEngine->drawLine(cx, aY, cx - aW, aY - aH);
         g_pRenderEngine->drawLine(cx, aY, cx + aW, aY - aH);
         g_pRenderEngine->drawLine(cx - aW, aY - aH, cx + aW, aY - aH);

         // Bubble background
         g_pRenderEngine->setFill(0.10f, 0.26f, 0.16f, 0.95f);
         g_pRenderEngine->setStroke(0.18f, 0.86f, 0.46f, 1.0f);
         g_pRenderEngine->setStrokeSize(2);
         g_pRenderEngine->drawRoundRect(bubX, bubY, bubW, bubH, 0.003f);

         // Text
         float th1 = g_pRenderEngine->textHeight(g_idFontMenu);
         g_pRenderEngine->setColors(get_Color_IconSucces());
         g_pRenderEngine->drawText(bubX + wPx * 8, bubY + hPx * 5, g_idFontMenu, szLn1);
         g_pRenderEngine->setColors(get_Color_MenuText());
         g_pRenderEngine->drawText(bubX + wPx * 8, bubY + th1 + hPx * 7, g_idFontMenuSmall, szLn2);
      }
   }

   // Highlight the channel currently being scanned
   if ( m_bScanInProgress && m_iChannelsScanned < m_iScanChannelsCount )
   {
      int curCh = m_iChannelsScanned;
      float bx0 = fGrX + curCh * barW + gap;
      float bx1 = fGrX + (curCh + 1) * barW - gap;
      g_pRenderEngine->setFill(0.28f, 0.48f, 0.88f, 0.25f);
      g_pRenderEngine->setStroke(0.28f, 0.48f, 0.88f, 0.75f);
      g_pRenderEngine->setStrokeSize(2);
      g_pRenderEngine->drawRect(bx0 - wPx, fGrY, (bx1 - bx0) + wPx * 2, fGrH);
   }
}

void MenuRFScan::onSelectItem()
{
   Menu::onSelectItem();
   if ( m_SelectedIndex < 0 || m_pMenuItems[m_SelectedIndex]->isEditing() )
      return;

   if ( m_SelectedIndex == m_IndexRescan )
   {
      _startScan();
      return;
   }

   if ( m_SelectedIndex == m_IndexUseRecommended )
   {
      if ( m_uBestFreqKhz == 0 )
         return;
      menu_stack_pop(1);
      return;
   }

   if ( m_SelectedIndex == m_IndexCancel )
   {
      menu_stack_pop(0);
      return;
   }
}

u32 menu_rf_scan_get_best_freq_khz()
{
   if ( s_pLastRFScanMenu )
      return s_pLastRFScanMenu->getBestFreqKhz();
   return 0;
}
