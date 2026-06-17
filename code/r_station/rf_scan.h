#pragma once
#include "../base/base.h"

void rf_scan_start(u32 uBandFlags);
void rf_scan_periodic_loop();
void rf_scan_stop();
bool rf_scan_is_in_progress();
