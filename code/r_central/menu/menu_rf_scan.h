#pragma once
#include "menu_objects.h"
#include "menu_item_text.h"

#define RF_SCAN_MAX_CHANNELS  128
#define RF_SCAN_RESULTS_FILE  "/tmp/ruby_rf_scan_results.txt"

// Per-channel busy ratio sentinels (must match the router side in rf_scan.cpp).
#define RF_SCAN_BUSY_NOT_SCANNED (-1000) // channel not measured yet
#define RF_SCAN_BUSY_INVALID     (-1)    // measured, but the card reported no usable survey data

class MenuRFScan : public Menu
{
   public:
      MenuRFScan(int iVehicleRadioLink, u32 uFreqKhz);
      virtual ~MenuRFScan();
      virtual void onShow();
      virtual void Render();
      virtual bool periodicLoop();
      virtual void onSelectItem();
      virtual int  onBack();

      u32 getBestFreqKhz() const { return m_uBestFreqKhz; }

   private:
      void _startScan();
      void _stopScan();
      void _readResultsFile();
      void _renderSpectrumChart(float xPos, float yPos, float fWidth, float fHeight);

      int  m_iVehicleRadioLink;
      u32  m_uCurrentFreqKhz;
      int  m_uBandFlags;

      u32  m_uScanChannels[RF_SCAN_MAX_CHANNELS];
      int  m_iScanChannelsCount;
      int  m_iBusyPct[RF_SCAN_MAX_CHANNELS];
      int  m_iChannelsScanned;
      u32  m_uBestFreqKhz;
      int  m_iBestChannelIndex;

      bool m_bScanInProgress;
      bool m_bScanComplete;
      bool m_bNoSurveyData;

      int  m_IndexRescan;
      int  m_IndexUseRecommended;
      int  m_IndexCancel;
};

u32 menu_rf_scan_get_best_freq_khz();
