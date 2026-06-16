#pragma once
#include "menu_objects.h"
#include "menu_item_text.h"

#define RF_SCAN_MAX_CHANNELS  128
#define RF_SCAN_RESULTS_FILE  "/tmp/ruby_rf_scan_results.txt"

class MenuRFScan : public Menu
{
   public:
      MenuRFScan(int iVehicleRadioLink, u32 uFreqKhz);
      virtual ~MenuRFScan();
      virtual void onShow();
      virtual void Render();
      virtual bool periodicLoop();
      virtual void onSelectItem();

      u32 getBestFreqKhz() const { return m_uBestFreqKhz; }

   private:
      void _startScan();
      void _readResultsFile();
      void _renderSpectrumChart(float xPos, float yPos, float fWidth, float fHeight);

      int  m_iVehicleRadioLink;
      u32  m_uCurrentFreqKhz;
      int  m_uBandFlags;

      u32  m_uScanChannels[RF_SCAN_MAX_CHANNELS];
      int  m_iScanChannelsCount;
      int  m_iNoiseDbm[RF_SCAN_MAX_CHANNELS];
      int  m_iChannelsScanned;
      u32  m_uBestFreqKhz;
      int  m_iBestChannelIndex;

      bool m_bScanInProgress;
      bool m_bScanComplete;

      int  m_IndexRescan;
      int  m_IndexUseRecommended;
      int  m_IndexCancel;
};

u32 menu_rf_scan_get_best_freq_khz();
