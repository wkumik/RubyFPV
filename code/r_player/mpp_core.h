#pragma once

#include "../base/base.h"
#include "../base/config.h"
#include "../base/hardware.h"
#include "../base/hardware_procs.h"
#include "../base/shared_mem.h"
#include "../renderer/drm_core.h"
#include <ctype.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <semaphore.h>

#include <linux/videodev2.h>
#include <rockchip/rk_mpi.h>

extern shared_mem_process_stats* g_pSMProcessStats;

// Max time (in ms) to keep retrying to feed a frame to the MPP decoder before
// giving up and signaling a real decoder-stall alarm. Short transient hiccups
// (a few ms) are normal decode backpressure and must NOT raise the alarm.
#define MPP_DECODER_GIVEUP_MS 100

int mpp_init(bool bUseH265Decoder, int iMPPBuffersSize, u32 uCPUAffinityMask, int iRawPriority);
int mpp_uninit();
void mpp_enable_vsync(bool bEnableVSync);

int mpp_start_decoding_thread();
int mpp_feed_data_to_decoder(void* pData, int iLength);
int mpp_mark_end_of_stream();
bool mpp_get_clear_stream_changed_flag();


