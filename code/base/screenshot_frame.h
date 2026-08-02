#pragma once

// Handoff format for the last displayed video frame, written by the video player
// process (which owns the decoded frames) and read by ruby_central (which owns the
// OSD pixels), so that a screenshot can contain both layers. The two are composited
// by the display hardware on separate DRM planes, so neither process has the full
// picture on its own.

#include "base.h"

#define SCREENSHOT_FRAME_MAGIC 0x52534346  // 'RSCF'
#define SCREENSHOT_FRAME_VERSION 1

typedef struct
{
   u32 uMagic;
   u32 uVersion;
   // NV12 plane geometry, as allocated by the decoder
   u32 uVideoWidth;    // visible width
   u32 uVideoHeight;   // visible height
   u32 uStrideH;       // bytes per row, both Y and UV planes
   u32 uStrideV;       // rows in the Y plane, UV plane starts at uStrideH * uStrideV
   // Where the video plane lands on the display, as computed for the DRM plane
   u32 uDestX;
   u32 uDestY;
   u32 uDestWidth;
   u32 uDestHeight;
} __attribute__((packed)) type_screenshot_frame_header;
