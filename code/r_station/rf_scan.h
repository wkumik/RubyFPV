#pragma once
#include "../base/base.h"

void rf_scan_start(u32 uBandFlags, int iFDToCentral);
void rf_scan_periodic_loop();
void rf_scan_stop();
bool rf_scan_is_in_progress();
u32  rf_scan_get_best_freq_khz();
