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

#include "../base/base.h"
#include "../base/config.h"
#include "../base/hardware.h"
#include "../base/hardware_procs.h"
#include "../common/string_utils.h"
#include "media.h"
#include "../renderer/render_engine.h"
#include "popup.h"
#include "ruby_central.h"
#include "shared_vars.h"
#include "timers.h"

#include <sys/types.h>
#include <dirent.h>
#include <string.h>
#include <pthread.h>
#include <signal.h>

#ifdef HW_PLATFORM_RADXA
#include <cairo.h>
#include "../base/screenshot_frame.h"
#include "../renderer/drm_core.h"
#endif

static int s_iScreenshotsCountOnDisk = 0;
static int s_iVideoCountOnDisk = 0;

static int s_iMediaBootCount = 0;
static int s_iMediaInitCount = 0;
static char s_szMediaCurrentScreenshotFileName[MAX_FILE_PATH_SIZE];
static char s_szMediaCurrentVideoFileInfo[MAX_FILE_PATH_SIZE];

void _media_remove_invalid_files()
{
   DIR *d;
   FILE* fd;
   struct dirent *dir;
   char szFile[MAX_FILE_PATH_SIZE];
   char szComm[1024];
   log_line("Searching and removing invalid media files...");
   d = opendir(FOLDER_MEDIA);
   if (d)
   {
      while ((dir = readdir(d)) != NULL)
      {
         if ( strlen(dir->d_name) < 4 )
            continue;

         snprintf(szFile, sizeof(szFile)/sizeof(szFile[0]), "%s%s", FOLDER_MEDIA, dir->d_name);
         long lSize = 0;
         fd = fopen(szFile, "rb");
         if ( NULL != fd )
         {
            fseek(fd, 0, SEEK_END);
            lSize = ftell(fd);
            fseek(fd, 0, SEEK_SET);
            fclose(fd);
         }
         else
            log_softerror_and_alarm("Failed to open file [%s] for checking it's size.", szFile);
         if ( lSize > 3 )
            continue;

         // Remove small files (less than 3 bytes)

         log_line("Removing invalid media file (size too small: %d bytes): [%s]", lSize, dir->d_name);

         // Remove invalid file and info file for it
         strcpy(szFile, dir->d_name);
         int pos = strlen(szFile);
         while (pos > 0 && szFile[pos] != '.')
            pos--;
         pos++;

         szFile[pos] = 0;
         strcat(szFile, "h26*");
         snprintf(szComm, sizeof(szComm)/sizeof(szComm[0]), "rm -rf %s%s", FOLDER_MEDIA, szFile);
         hw_execute_bash_command(szComm, NULL);

         szFile[pos] = 0;
         strcat(szFile, "info");
         snprintf(szComm, sizeof(szComm)/sizeof(szComm[0]), "rm -rf %s%s", FOLDER_MEDIA, szFile);
         hw_execute_bash_command(szComm, NULL);

         szFile[pos] = 0;
         strcat(szFile, "png");
         snprintf(szComm, sizeof(szComm)/sizeof(szComm[0]), "rm -rf %s%s", FOLDER_MEDIA, szFile);
         hw_execute_bash_command(szComm, NULL);
         ruby_signal_alive();
      }
      closedir(d);
   }
   else
      log_softerror_and_alarm("Failed to open media dir to search for invalid videos.");
   log_line("Searching and removing invalid media files complete.");
}


bool media_init_and_scan()
{
   log_line("Media Storage: Init media storage...");
   ruby_pause_watchdog("scanning media");

   s_iMediaBootCount = 0;
   s_szMediaCurrentScreenshotFileName[0] = 0;
   s_szMediaCurrentVideoFileInfo[0] = 0;
   s_iMediaInitCount++;

   char szFile[MAX_FILE_PATH_SIZE];
   strcpy(szFile, FOLDER_CONFIG);
   strcat(szFile, FILE_CONFIG_BOOT_COUNT);
   FILE* fd = fopen(szFile, "r");
   if ( NULL != fd )
   {
      if ( 1 != fscanf(fd, "%d", &s_iMediaBootCount) )
         s_iMediaBootCount = 0;
      fclose(fd);
   }

   media_scan_files();

   ruby_resume_watchdog("scanning media");
   log_line("Media Storage: Init media storage completed.");

   return true;
}

void media_scan_files()
{
   char szOutBuff[1024];

   _media_remove_invalid_files();
 

   // Count files in a folder:
   // ls media/ | grep picture- | wc -l
   // ls media/ | grep '.info' | wc -l

   s_iScreenshotsCountOnDisk = 0;
   szOutBuff[0] = 0;
   hw_execute_bash_command("ls media/ | grep picture- | wc -l", szOutBuff);
   if ( 0 < strlen(szOutBuff) )
      s_iScreenshotsCountOnDisk = strtol(szOutBuff, NULL, 10);

   s_iVideoCountOnDisk = 0;
   szOutBuff[0] = 0;
   hw_execute_bash_command("ls media/ | grep '.info' | wc -l", szOutBuff);
   if ( 0 < strlen(szOutBuff) )
      s_iVideoCountOnDisk = strtol(szOutBuff, NULL, 10);

   ruby_signal_alive();

   log_line("Media storage: Found %d screenshots on storage.", s_iScreenshotsCountOnDisk );
   log_line("Media storage: Found %d videos on storage.", s_iVideoCountOnDisk );
}

int media_get_screenshots_count()
{
   return s_iScreenshotsCountOnDisk;
}

int media_get_videos_count()
{
   return s_iVideoCountOnDisk;
}


char* media_get_screenshot_filename()
{
   char vehicle_name[MAX_VEHICLE_NAME_LENGTH+1];

   strcpy(vehicle_name, "none");
   if ( NULL != g_pCurrentModel )
      strcpy(vehicle_name, g_pCurrentModel->vehicle_name);
   if ( (0 == strlen(vehicle_name)) || (1 == strlen(vehicle_name) && vehicle_name[0] == ' ') )
      strcpy(vehicle_name, "none");
    
   str_sanitize_filename(vehicle_name);

   sprintf(s_szMediaCurrentScreenshotFileName, FILE_FORMAT_SCREENSHOT, vehicle_name, s_iMediaBootCount, g_TimeNow/1000, g_TimeNow%1000 );
   return s_szMediaCurrentScreenshotFileName;
}



char* media_get_video_filename()
{
   char vehicle_name[MAX_VEHICLE_NAME_LENGTH+1];

   strcpy(vehicle_name, "none");
   if ( NULL != g_pCurrentModel )
      strcpy(vehicle_name, g_pCurrentModel->vehicle_name);
   if ( (0 == strlen(vehicle_name)) || (1 == strlen(vehicle_name) && vehicle_name[0] == ' ') )
      strcpy(vehicle_name, "none");

   str_sanitize_filename(vehicle_name);

   sprintf(s_szMediaCurrentVideoFileInfo, FILE_FORMAT_VIDEO_INFO, vehicle_name, s_iMediaBootCount, g_TimeNow/1000, g_TimeNow%1000 );
   return s_szMediaCurrentVideoFileInfo;
}

static char s_szMediaScreenShotFilename[MAX_FILE_PATH_SIZE];
static bool s_bMediaIsTakingScreenShot = false;
static pthread_t s_pThreadMediaTakeScreenShot;

#ifdef HW_PLATFORM_RADXA

// On Radxa the video and the OSD live on two separate DRM planes, in two separate
// processes, and are only composited by the display hardware at scan out time. So a
// screenshot has to be built by hand: we keep a copy of our own OSD buffer, ask the
// video player for the frame it currently has on screen, and blend the two here.

// The player's display loop waits up to a second on its frame semaphore, so give it
// more than that: on a stalled video stream the request is only picked up after that
// wait expires.
#define SCREENSHOT_VIDEO_FRAME_TIMEOUT_MS 1500

static u8* s_pMediaScreenshotOSD = NULL;
static int s_iMediaScreenshotOSDWidth = 0;
static int s_iMediaScreenshotOSDHeight = 0;
static int s_iMediaScreenshotOSDStride = 0;

static void _media_screenshot_get_frame_file_name(char* szFile, int iMaxLength)
{
   snprintf(szFile, iMaxLength, "%s%s", FOLDER_RUBY_TEMP, FILE_TEMP_SCREENSHOT_FRAME);
}

// Keeps a copy of what is on the OSD plane right now. Must be called from the render
// loop's thread: the front buffer is stable there, the back one is being drawn into.
static bool _media_screenshot_copy_osd_buffer()
{
   type_drm_buffer* pBuffer = ruby_drm_core_get_main_draw_buffer();
   if ( (NULL == pBuffer) || (NULL == pBuffer->pData) || (0 == pBuffer->uSize) )
   {
      log_softerror_and_alarm("Media Storage: No OSD buffer to copy for screenshot.");
      return false;
   }

   if ( NULL != s_pMediaScreenshotOSD )
      free(s_pMediaScreenshotOSD);
   s_pMediaScreenshotOSD = (u8*) malloc(pBuffer->uSize);
   if ( NULL == s_pMediaScreenshotOSD )
   {
      log_softerror_and_alarm("Media Storage: Failed to allocate %u bytes for screenshot OSD buffer.", pBuffer->uSize);
      return false;
   }

   memcpy(s_pMediaScreenshotOSD, pBuffer->pData, pBuffer->uSize);
   s_iMediaScreenshotOSDWidth = (int)pBuffer->uWidth;
   s_iMediaScreenshotOSDHeight = (int)pBuffer->uHeight;
   s_iMediaScreenshotOSDStride = (int)pBuffer->uStride;
   return true;
}

static void _media_screenshot_request_video_frame()
{
   char szFile[MAX_FILE_PATH_SIZE];
   _media_screenshot_get_frame_file_name(szFile, sizeof(szFile)/sizeof(szFile[0]));
   unlink(szFile);

   char szPIDs[256];
   szPIDs[0] = 0;
   hw_process_get_pids(VIDEO_PLAYER_SM, szPIDs);
   removeTrailingNewLines(szPIDs);

   int iCountSignaled = 0;
   char* pszPID = strtok(szPIDs, " \n\r");
   while ( NULL != pszPID )
   {
      int iPID = atoi(pszPID);
      if ( iPID > 0 )
      if ( 0 == kill(iPID, SIGUSR1) )
         iCountSignaled++;
      pszPID = strtok(NULL, " \n\r");
   }

   if ( 0 == iCountSignaled )
      log_line("Media Storage: No video player to ask for a screenshot frame, will screenshot the OSD only.");
}

// Returns the NV12 frame the player wrote out, or NULL if it did not produce one in
// time (no video, or player not running). Caller frees the returned buffer.
static u8* _media_screenshot_wait_video_frame(type_screenshot_frame_header* pHeader)
{
   char szFile[MAX_FILE_PATH_SIZE];
   _media_screenshot_get_frame_file_name(szFile, sizeof(szFile)/sizeof(szFile[0]));

   u32 uTimeEnd = get_current_timestamp_ms() + SCREENSHOT_VIDEO_FRAME_TIMEOUT_MS;
   FILE* fd = NULL;
   while ( get_current_timestamp_ms() < uTimeEnd )
   {
      fd = fopen(szFile, "rb");
      if ( NULL != fd )
         break;
      hardware_sleep_ms(10);
   }
   if ( NULL == fd )
   {
      log_line("Media Storage: Timed out waiting for a video frame for the screenshot.");
      return NULL;
   }

   u8* pData = NULL;
   if ( 1 != fread(pHeader, sizeof(type_screenshot_frame_header), 1, fd) )
      log_softerror_and_alarm("Media Storage: Failed to read screenshot video frame header.");
   else if ( (SCREENSHOT_FRAME_MAGIC != pHeader->uMagic) || (SCREENSHOT_FRAME_VERSION != pHeader->uVersion) )
      log_softerror_and_alarm("Media Storage: Unexpected screenshot video frame format (magic %u, version %u).", pHeader->uMagic, pHeader->uVersion);
   else if ( (pHeader->uVideoWidth < 2) || (pHeader->uVideoHeight < 2) ||
             (pHeader->uStrideH < pHeader->uVideoWidth) || (pHeader->uStrideV < pHeader->uVideoHeight) )
      log_softerror_and_alarm("Media Storage: Invalid screenshot video frame size (%u x %u, strides %u x %u).",
         pHeader->uVideoWidth, pHeader->uVideoHeight, pHeader->uStrideH, pHeader->uStrideV);
   else
   {
      u32 uSize = pHeader->uStrideH * pHeader->uStrideV;
      uSize += pHeader->uStrideH * (pHeader->uVideoHeight/2);
      pData = (u8*) malloc(uSize);
      if ( NULL == pData )
         log_softerror_and_alarm("Media Storage: Failed to allocate %u bytes for screenshot video frame.", uSize);
      else if ( uSize != fread(pData, 1, uSize, fd) )
      {
         log_softerror_and_alarm("Media Storage: Failed to read the screenshot video frame data.");
         free(pData);
         pData = NULL;
      }
   }
   fclose(fd);
   unlink(szFile);
   return pData;
}

// NV12 (limited range BT.709, as the decoder delivers it) to the packed 32 bit RGB
// that Cairo wants. Fixed point, this runs over a couple million pixels.
static void _media_screenshot_convert_nv12(u8* pFrame, type_screenshot_frame_header* pHeader, u8* pOut, int iOutStride)
{
   int iWidth = (int)pHeader->uVideoWidth;
   int iHeight = (int)pHeader->uVideoHeight;
   int iStride = (int)pHeader->uStrideH;
   u8* pPlaneY = pFrame;
   u8* pPlaneUV = pFrame + iStride * (int)pHeader->uStrideV;

   for( int y=0; y<iHeight; y++ )
   {
      u8* pRowY = pPlaneY + y * iStride;
      u8* pRowUV = pPlaneUV + (y/2) * iStride;
      u32* pRowOut = (u32*)(pOut + y * iOutStride);
      for( int x=0; x<iWidth; x++ )
      {
         int iY = (((int)pRowY[x]) - 16) * 1192;
         int iU = ((int)pRowUV[(x & ~1)]) - 128;
         int iV = ((int)pRowUV[(x & ~1) + 1]) - 128;

         int iR = (iY + 1836 * iV) >> 10;
         int iG = (iY - 218 * iU - 546 * iV) >> 10;
         int iB = (iY + 2163 * iU) >> 10;

         if ( iR < 0 ) iR = 0; else if ( iR > 255 ) iR = 255;
         if ( iG < 0 ) iG = 0; else if ( iG > 255 ) iG = 255;
         if ( iB < 0 ) iB = 0; else if ( iB > 255 ) iB = 255;

         pRowOut[x] = 0xFF000000 | (((u32)iR)<<16) | (((u32)iG)<<8) | ((u32)iB);
      }
   }
}

// Paints the video frame where it sits on screen, then the OSD on top of it, and
// writes the result out. Either layer can be missing.
static bool _media_screenshot_compose_and_save(const char* szFile, u8* pFrame, type_screenshot_frame_header* pHeader, bool bIncludeOSD)
{
   type_drm_display_attributes* pDisplayInfo = ruby_drm_get_main_display_info();
   if ( NULL == pDisplayInfo )
      return false;
   int iWidth = pDisplayInfo->iWidth;
   int iHeight = pDisplayInfo->iHeight;
   if ( (iWidth < 2) || (iHeight < 2) )
      return false;

   cairo_surface_t* pOutputSurface = cairo_image_surface_create(CAIRO_FORMAT_RGB24, iWidth, iHeight);
   if ( CAIRO_STATUS_SUCCESS != cairo_surface_status(pOutputSurface) )
   {
      log_softerror_and_alarm("Media Storage: Failed to create the screenshot surface.");
      cairo_surface_destroy(pOutputSurface);
      return false;
   }
   cairo_t* pCairoCtx = cairo_create(pOutputSurface);

   // Anything not covered by the video is black, same as on screen
   cairo_set_source_rgb(pCairoCtx, 0, 0, 0);
   cairo_paint(pCairoCtx);

   cairo_surface_t* pVideoSurface = NULL;
   if ( NULL != pFrame )
   {
      pVideoSurface = cairo_image_surface_create(CAIRO_FORMAT_RGB24, (int)pHeader->uVideoWidth, (int)pHeader->uVideoHeight);
      if ( CAIRO_STATUS_SUCCESS != cairo_surface_status(pVideoSurface) )
      {
         log_softerror_and_alarm("Media Storage: Failed to create the screenshot video surface.");
         cairo_surface_destroy(pVideoSurface);
         pVideoSurface = NULL;
      }
      else
      {
         cairo_surface_flush(pVideoSurface);
         _media_screenshot_convert_nv12(pFrame, pHeader, cairo_image_surface_get_data(pVideoSurface), cairo_image_surface_get_stride(pVideoSurface));
         cairo_surface_mark_dirty(pVideoSurface);

         int iDestWidth = (int)pHeader->uDestWidth;
         int iDestHeight = (int)pHeader->uDestHeight;
         if ( (iDestWidth < 2) || (iDestHeight < 2) )
         {
            iDestWidth = iWidth;
            iDestHeight = iHeight;
         }
         cairo_save(pCairoCtx);
         cairo_translate(pCairoCtx, (double)pHeader->uDestX, (double)pHeader->uDestY);
         cairo_scale(pCairoCtx, (double)iDestWidth/(double)pHeader->uVideoWidth, (double)iDestHeight/(double)pHeader->uVideoHeight);
         cairo_set_source_surface(pCairoCtx, pVideoSurface, 0, 0);
         cairo_paint(pCairoCtx);
         cairo_restore(pCairoCtx);
      }
   }

   if ( bIncludeOSD && (NULL != s_pMediaScreenshotOSD) )
   {
      cairo_surface_t* pOSDSurface = cairo_image_surface_create_for_data(s_pMediaScreenshotOSD, CAIRO_FORMAT_ARGB32,
         s_iMediaScreenshotOSDWidth, s_iMediaScreenshotOSDHeight, s_iMediaScreenshotOSDStride);
      if ( CAIRO_STATUS_SUCCESS != cairo_surface_status(pOSDSurface) )
         log_softerror_and_alarm("Media Storage: Failed to create the screenshot OSD surface.");
      else
      {
         cairo_set_source_surface(pCairoCtx, pOSDSurface, 0, 0);
         cairo_paint(pCairoCtx);
      }
      cairo_surface_destroy(pOSDSurface);
   }

   cairo_destroy(pCairoCtx);
   cairo_status_t status = cairo_surface_write_to_png(pOutputSurface, szFile);
   cairo_surface_destroy(pOutputSurface);
   if ( NULL != pVideoSurface )
      cairo_surface_destroy(pVideoSurface);

   if ( CAIRO_STATUS_SUCCESS != status )
   {
      log_softerror_and_alarm("Media Storage: Failed to write screenshot [%s]: %s", szFile, cairo_status_to_string(status));
      return false;
   }
   return true;
}

static void* _thread_media_take_screenshot_radxa(void *argument)
{
   hw_log_current_thread_attributes("take screenshot");
   bool bIncludeOSD = (NULL != argument);

   type_screenshot_frame_header header;
   memset(&header, 0, sizeof(header));
   u8* pFrame = _media_screenshot_wait_video_frame(&header);

   bool bSaved = _media_screenshot_compose_and_save(s_szMediaScreenShotFilename, pFrame, &header, bIncludeOSD);

   if ( NULL != pFrame )
      free(pFrame);
   if ( NULL != s_pMediaScreenshotOSD )
      free(s_pMediaScreenshotOSD);
   s_pMediaScreenshotOSD = NULL;

   if ( bSaved )
   {
      log_line("Media Storage: Took a screenshot to file: %s", s_szMediaScreenShotFilename);
      s_iScreenshotsCountOnDisk++;
      Popup* p = new Popup("Screenshot taken", 0.1,0.72, 2);
      popups_add_topmost(p);
   }
   else
   {
      Popup* p = new Popup("Failed to take screenshot", 0.1,0.72, 3);
      popups_add_topmost(p);
   }

   s_bMediaIsTakingScreenShot = false;
   return NULL;
}

#endif

void _media_take_screenshot()
{
   if ( 0 == s_szMediaCurrentScreenshotFileName[0] )
      return;

   s_bMediaIsTakingScreenShot = true;
   char szComm[256];
   sprintf(szComm, "./raspi2png -p %s", s_szMediaScreenShotFilename);
   hw_execute_bash_command_nonblock(szComm, NULL);
   ruby_signal_alive();

   log_line("Media Storage: Took a screenshot to file: %s", s_szMediaScreenShotFilename);
   s_iScreenshotsCountOnDisk++;

   Popup* p = new Popup("Screenshot taken", 0.1,0.72, 2);
   popups_add_topmost(p);
   s_bMediaIsTakingScreenShot = false;
}

static void * _thread_media_take_screenshot(void *argument)
{
   hw_log_current_thread_attributes("take screenshot");
   _media_take_screenshot();
   return NULL;
}


bool media_take_screenshot(bool bIncludeOSD)
{
   if ( s_bMediaIsTakingScreenShot )
      return false;

   #ifndef HW_PLATFORM_RADXA
   if ( ! bIncludeOSD )
   {
      g_pRenderEngine->startFrame();
      g_pRenderEngine->endFrame();
      hardware_sleep_ms(25);
   }
   #endif

   media_get_screenshot_filename();

   char szFile[MAX_FILE_PATH_SIZE];
   strcpy(szFile, FOLDER_MEDIA);
   strcat(szFile, s_szMediaCurrentScreenshotFileName);
   strncpy(s_szMediaScreenShotFilename, szFile, MAX_FILE_PATH_SIZE);

   #ifdef HW_PLATFORM_RADXA
   log_line("Media Storage: Taking screenshot to file: %s", szFile);
   s_bMediaIsTakingScreenShot = true;

   // Grab the OSD before we go async, the render loop keeps overwriting it
   if ( bIncludeOSD )
   if ( ! _media_screenshot_copy_osd_buffer() )
      bIncludeOSD = false;

   _media_screenshot_request_video_frame();
   ruby_signal_alive();

   if ( 0 != pthread_create(&s_pThreadMediaTakeScreenShot, NULL, &_thread_media_take_screenshot_radxa, bIncludeOSD?(void*)1:NULL) )
   {
      log_softerror_and_alarm("Media Storage: Failed to start the screenshot thread.");
      if ( NULL != s_pMediaScreenshotOSD )
         free(s_pMediaScreenshotOSD);
      s_pMediaScreenshotOSD = NULL;
      s_bMediaIsTakingScreenShot = false;
      return false;
   }
   pthread_detach(s_pThreadMediaTakeScreenShot);
   return true;
   #endif

   ruby_signal_alive();

   if ( 0 != pthread_create(&s_pThreadMediaTakeScreenShot, NULL, &_thread_media_take_screenshot, NULL) )
      _media_take_screenshot();
   else
      pthread_detach(s_pThreadMediaTakeScreenShot);
   return true;
}
