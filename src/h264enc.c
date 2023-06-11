/*
 * Copyright (c) 2014-2015 Jens Kuske <jenskuske@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 *
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "vencoder.h"
#include <sys/time.h>
#include <time.h>
#include <memoryAdapter.h>
#include <math.h>
#include <stdbool.h>
#include <time.h>
#include "h264enc.h"

// #define DO_LATENCY_TEST
 
#define MSG(x) fprintf(stderr, "h264enc: " x "\n")
//#define PMSG(x) fprintf(stderr, "Pete: " x "\n"); fflush(stderr)
#define PMSG(x)
#define ROI_NUM 4
#define ALIGN_XXB(y, x) (((x) + ((y)-1)) & ~((y)-1))

typedef struct {
    unsigned int width;
    unsigned int height;
    unsigned int width_aligh16;
    unsigned int height_aligh16;
    unsigned char* argb_addr;
    unsigned int size;
}BitMapInfoS;


struct h264enc_internal {
    VencHeaderData          sps_pps_data;
    VencH264Param           h264Param;
    VencMBModeCtrl          h264MBMode;
    VencMBInfo              MBInfo;
    VencH264FixQP           fixQP;
    VencSuperFrameConfig    sSuperFrameCfg;
    VencH264SVCSkip         SVCSkip; // set SVC and skip_frame
    VencH264AspectRatio     sAspectRatio;
    VencH264VideoSignal     sVideoSignal;
    VencCyclicIntraRefresh  sIntraRefresh;
    VencROIConfig           sRoiConfig[ROI_NUM];
    VeProcSet               sVeProcInfo;
    VencSmartFun            sH264Smart;

    VencBaseConfig baseConfig;
    VencAllocateBufferParam bufferParam;
    VideoEncoder* pVideoEnc;
    VencInputBuffer inputBuffer;
    VencOutputBuffer outputBuffer;
    long long pts;
};


#ifdef DO_LATENCY_TEST
long long GetNowUs()
{
    struct timeval now;
    gettimeofday(&now, NULL);
    return now.tv_sec * 1000000 + now.tv_usec;
}
#define NUM_TIME_STEPS 6
uint32_t FrameTimes[NUM_TIME_STEPS] = {0};
#endif


void init_mb_mode(VencMBModeCtrl *pMBMode, int width, int height)
{
    unsigned int mb_num;
    unsigned int j;

    mb_num = (ALIGN_XXB(16, width) >> 4)
                * (ALIGN_XXB(16, height) >> 4);
    pMBMode->p_info = malloc(sizeof(VencMBModeCtrlInfo) * mb_num);
    pMBMode->mode_ctrl_en = 1;

    for (j = 0; j < mb_num / 2; j++)
    {
        pMBMode->p_info[j].mb_en = 1;
        pMBMode->p_info[j].mb_skip_flag = 0;
        pMBMode->p_info[j].mb_qp = 22;
    }
    for (; j < mb_num; j++)
    {
        pMBMode->p_info[j].mb_en = 1;
        pMBMode->p_info[j].mb_skip_flag = 0;
        pMBMode->p_info[j].mb_qp = 32;
    }
}

void init_mb_info(VencMBInfo *MBInfo, int width, int height)
{
    MBInfo->num_mb = (ALIGN_XXB(16, width) *
                        ALIGN_XXB(16, height)) >> 8;
    MBInfo->p_para = (VencMBInfoPara *)malloc(sizeof(VencMBInfoPara) * MBInfo->num_mb);
    if(MBInfo->p_para == NULL)
    {
        MSG("malloc MBInfo->p_para error\n");
        return;
    }
   // logv("mb_num:%d, mb_info_queue_addr:%p\n", MBInfo->num_mb, MBInfo->p_para);
}


void init_fix_qp(VencH264FixQP *fixQP)
{
    fixQP->bEnable = 1;
    fixQP->nIQp = 35;
    fixQP->nPQp = 35;
}

void init_super_frame_cfg(VencSuperFrameConfig *sSuperFrameCfg)
{
    sSuperFrameCfg->eSuperFrameMode = VENC_SUPERFRAME_NONE;
    sSuperFrameCfg->nMaxIFrameBits = 30000*8;
    sSuperFrameCfg->nMaxPFrameBits = 15000*8;
}

void init_svc_skip(VencH264SVCSkip *SVCSkip)
{
    SVCSkip->nTemporalSVC = T_LAYER_4;
    switch(SVCSkip->nTemporalSVC)
    {
        case T_LAYER_4:
            SVCSkip->nSkipFrame = SKIP_8;
            break;
        case T_LAYER_3:
            SVCSkip->nSkipFrame = SKIP_4;
            break;
        case T_LAYER_2:
            SVCSkip->nSkipFrame = SKIP_2;
            break;
        default:
            SVCSkip->nSkipFrame = NO_SKIP;
            break;
    }
}

void init_aspect_ratio(VencH264AspectRatio *sAspectRatio)
{
    sAspectRatio->aspect_ratio_idc = 255;
    sAspectRatio->sar_width = 4;
    sAspectRatio->sar_height = 3;
}

void init_video_signal(VencH264VideoSignal *sVideoSignal)
{
    sVideoSignal->video_format = 5;
    sVideoSignal->src_colour_primaries = 0;
    sVideoSignal->dst_colour_primaries = 1;
}

void init_intra_refresh(VencCyclicIntraRefresh *sIntraRefresh)
{
    sIntraRefresh->bEnable = 1;
    sIntraRefresh->nBlockNumber = 10;
}

void init_roi(VencROIConfig *sRoiConfig)
{
    sRoiConfig[0].bEnable = 1;
    sRoiConfig[0].index = 0;
    sRoiConfig[0].nQPoffset = 10;
    sRoiConfig[0].sRect.nLeft = 0;
    sRoiConfig[0].sRect.nTop = 0;
    sRoiConfig[0].sRect.nWidth = 1280;
    sRoiConfig[0].sRect.nHeight = 320;

    sRoiConfig[1].bEnable = 1;
    sRoiConfig[1].index = 1;
    sRoiConfig[1].nQPoffset = 10;
    sRoiConfig[1].sRect.nLeft = 320;
    sRoiConfig[1].sRect.nTop = 180;
    sRoiConfig[1].sRect.nWidth = 320;
    sRoiConfig[1].sRect.nHeight = 180;

    sRoiConfig[2].bEnable = 1;
    sRoiConfig[2].index = 2;
    sRoiConfig[2].nQPoffset = 10;
    sRoiConfig[2].sRect.nLeft = 320;
    sRoiConfig[2].sRect.nTop = 180;
    sRoiConfig[2].sRect.nWidth = 320;
    sRoiConfig[2].sRect.nHeight = 180;

    sRoiConfig[3].bEnable = 1;
    sRoiConfig[3].index = 3;
    sRoiConfig[3].nQPoffset = 10;
    sRoiConfig[3].sRect.nLeft = 320;
    sRoiConfig[3].sRect.nTop = 180;
    sRoiConfig[3].sRect.nWidth = 320;
    sRoiConfig[3].sRect.nHeight = 180;
}

void init_enc_proc_info(VeProcSet *ve_proc_set)
{
    ve_proc_set->bProcEnable = 1;
    ve_proc_set->nProcFreq = 3;
}

void releaseMb(h264enc *h264_func)
{
    VencMBInfo *pMBInfo = NULL;
    VencMBModeCtrl *pMBMode = NULL;
    if(h264_func->h264MBMode.mode_ctrl_en)
    {
        pMBInfo = &h264_func->MBInfo;
        pMBMode = &h264_func->h264MBMode;
    }
    else
        return;

    if(pMBInfo->p_para)
        free(pMBInfo->p_para);
    if(pMBMode->p_info)
        free(pMBMode->p_info);
}

void initH264ParamsDefault(h264enc *h264_func)
{
    memset(h264_func, 0, sizeof(h264enc));

    //init h264Param
    h264_func->h264Param.bEntropyCodingCABAC = 1;
    h264_func->h264Param.nBitrate = 5 * 1024 * 1024; //FIXME
    h264_func->h264Param.nFramerate = 60;
    h264_func->h264Param.nCodingMode = VENC_FRAME_CODING;
    h264_func->h264Param.nMaxKeyInterval = 16; // FIXME
    h264_func->h264Param.sProfileLevel.nProfile = VENC_H264ProfileHigh; //VENC_H264ProfileHigh; / VENC_H264ProfileBaseline
    h264_func->h264Param.sProfileLevel.nLevel = VENC_H264Level41; // VENC_H264Level51;
    h264_func->h264Param.sQPRange.nMinqp = 10;
    h264_func->h264Param.sQPRange.nMaxqp = 50;
    h264_func->h264Param.bLongRefEnable = 1;
    h264_func->h264Param.nLongRefPoc = 0;

    h264_func->sH264Smart.img_bin_en = 1;
    h264_func->sH264Smart.img_bin_th = 27;
    h264_func->sH264Smart.shift_bits = 2;
    h264_func->sH264Smart.smart_fun_en = 1;
}

int initH264Func(h264enc *h264_func, int width, int height)
{
    //init VencMBModeCtrl
    init_mb_mode(&h264_func->h264MBMode, width, height);

    //init VencMBInfo
    init_mb_info(&h264_func->MBInfo, width, height);

    //init VencH264FixQP
    init_fix_qp(&h264_func->fixQP);

    //init VencSuperFrameConfig
    init_super_frame_cfg(&h264_func->sSuperFrameCfg);

    //init VencH264SVCSkip
    init_svc_skip(&h264_func->SVCSkip);

    //init VencH264AspectRatio
    init_aspect_ratio(&h264_func->sAspectRatio);

    //init VencH264AspectRatio
    init_video_signal(&h264_func->sVideoSignal);

    //init CyclicIntraRefresh
    init_intra_refresh(&h264_func->sIntraRefresh);

    //init VencROIConfig
    init_roi(h264_func->sRoiConfig);

    //init proc info
    init_enc_proc_info(&h264_func->sVeProcInfo);


    return 0;
}


static h264enc H264Enc = {0};
void h264enc_free(h264enc *c)
{
    if(c->pVideoEnc)
    {
        VideoEncDestroy(c->pVideoEnc);
    }
    c->pVideoEnc = NULL;

    
    if(H264Enc.baseConfig.memops)
    {
        CdcMemClose(H264Enc.baseConfig.memops);
    }
    releaseMb(c);
}
    
h264enc *h264enc_new(const struct h264enc_params *p)
{
    bool result;
    
    PMSG("h264enc_new()");
    
    initH264ParamsDefault(&H264Enc);

    H264Enc.h264Param.nBitrate = p->bitrate; 
   // H264Enc.h264Param.sQPRange.nMaxqp = p->qp; 
   // H264Enc.h264Param.sQPRange.nMinqp = p->qp - 1; 
    H264Enc.h264Param.nMaxKeyInterval = p->keyframe_interval; 
   
    fprintf(stderr, "bitrate=%d, qp=%d, keyframe=%d\n", p->bitrate, p->qp, p->keyframe_interval);
            
    memset(&(H264Enc.baseConfig), 0 ,sizeof(VencBaseConfig));
    memset(&(H264Enc.bufferParam), 0 ,sizeof(VencAllocateBufferParam));
    H264Enc.baseConfig.memops = MemAdapterGetOpsS();
    if (H264Enc.baseConfig.memops == NULL)
    {
        MSG("MemAdapterGetOpsS failed\n");
        return false;
    }
    
    
    CdcMemOpen(H264Enc.baseConfig.memops);
    H264Enc.baseConfig.nInputWidth= p->width;
    H264Enc.baseConfig.nInputHeight = p->height;
    H264Enc.baseConfig.nStride = p->width;
    H264Enc.baseConfig.nDstWidth = p->width;
    H264Enc.baseConfig.nDstHeight = p->height;
    
    H264Enc.baseConfig.eInputFormat = VENC_PIXEL_YUV420SP;
    
    H264Enc.bufferParam.nSizeY = H264Enc.baseConfig.nInputWidth*H264Enc.baseConfig.nInputHeight;
    H264Enc.bufferParam.nSizeC = H264Enc.baseConfig.nInputWidth*H264Enc.baseConfig.nInputHeight/2;
    H264Enc.bufferParam.nBufferNum = 1;
    
    H264Enc.pVideoEnc = VideoEncCreate(VENC_CODEC_H264);
    
    result = initH264Func(&H264Enc, p->width, p->height);
    if(result)
    {
        MSG("initH264Func error, return \n");
        return false;
    }

    unsigned int vbv_size = 12*1024*1024;
    VideoEncSetParameter(H264Enc.pVideoEnc, VENC_IndexParamH264Param, &(H264Enc.h264Param));
    fprintf(stderr, "Pete bitrate=%d, qp=%d, keyframe=%d\n", H264Enc.h264Param.nBitrate, H264Enc.h264Param.sQPRange.nMaxqp, H264Enc.h264Param.nMaxKeyInterval);
    VideoEncSetParameter(H264Enc.pVideoEnc, VENC_IndexParamSetVbvSize, &vbv_size);
    
    VideoEncInit(H264Enc.pVideoEnc, &(H264Enc.baseConfig));

    VideoEncGetParameter(H264Enc.pVideoEnc, VENC_IndexParamH264SPSPPS, &(H264Enc.sps_pps_data));
    
    // FIXME - we need to do something with this
    //fwrite(sps_pps_data.pBuffer, 1, sps_pps_data.nLength, out_file);
    
    AllocInputBuffer(H264Enc.pVideoEnc, &(H264Enc.bufferParam));
    PMSG("h264enc_new() complete");
	return &H264Enc;
}

int h264enc_get_initial_bytestream_length(h264enc *c)
{
    return c->sps_pps_data.nLength;
}


void *h264enc_get_intial_bytestream_buffer(h264enc *c)
{
   // fprintf(stderr, "Returning initial buffer of %d bytes\n", c->sps_pps_data.nLength);
    return c->sps_pps_data.pBuffer;
}

int FrameCTime = 0;


// Also need to update the init in SetLED()
#define NUM_LEDS 2
#define LED_STR_LEN 60
static const char LEDS[NUM_LEDS][LED_STR_LEN] = 
{  
    "/sys/class/gpio/gpio11/value", 
    "/sys/class/gpio/gpio12/value"
};

void SetLED(int LED, bool On)
{
   static bool HaveInited = false;
    
    if(!HaveInited)
    {
        FILE *fp = fopen("/sys/class/gpio/export", "w");
        if(!fp)
        {
            printf("Could not initialise LEDs\n"); 
            return;
        }
         fprintf(fp, "11");
        fclose(fp);
        fp = fopen("/sys/class/gpio/export", "w");
         fprintf(fp, "12");
        fclose(fp);
        fp = fopen("/sys/class/gpio/gpio11/direction", "w");
        if(!fp)
        {
            printf("Could not initialise LED 11\n"); 
            return;
        }
        fprintf(fp, "out");
        fclose(fp);
        fp = fopen("/sys/class/gpio/gpio12/direction", "w");
        if(!fp)
        {
            printf("Could not initialise LED 12\n"); 
            return;
        }
        fprintf(fp, "out");
        fclose(fp);
        HaveInited = true;
    }
   if(LED < NUM_LEDS)
   {
       FILE *fp = fopen(LEDS[LED], "w");
       if(fp)
       {
           fprintf(fp, "%d", (int)(!On));
           fclose(fp);
       }
       else
       {
           printf("Could not open [%s]\n", LEDS[LED]);
       }
   }
   else
   {
       printf("Invalid LED\n");
   }
}

bool LEDFrame = false;

#define WIDTH 1280
#define HEIGHT 720
bool DetectLEDFrame(unsigned char *Data, size_t Len)
{
    static int Count = 0;
    uint64_t TotLum = 0;
    uint32_t AvLum;
    int WindowSize = 8; // -WindowSize to +WindowSize
    int StartY = (HEIGHT / 2) - WindowSize;
    int StartX = (WIDTH / 2) - WindowSize;

    /* Make sure we can't retrigger within 0.3s */
    if(Count <= 18)
    {
        Count ++;
        return false;
    }
    for(int y = StartY; y < (StartY + WindowSize); y ++)
    {
        for(int x = StartX; x < (StartX + WindowSize); x ++)
        {
            int Elem = y * WIDTH + x;
            if(Elem >= Len)
            {
                printf("DetectLEDFrame() Invalid Elem %d, y=%d, x=%d\n", Elem, y, x);
            }
            else
            {
               TotLum += *(Data + Elem);
            }
        }
    }
    AvLum = TotLum / (WindowSize * WindowSize);
    //printf("AvLum=%d\n", AvLum);
    if(AvLum > 128)
    {
        Count = 0;
        return true;
    }
    return false;
}
void h264enc_set_input_buffer(h264enc *c, void *Dat, size_t Len)
{
    PMSG("h264enc_get_input_buffer()");

    #ifdef DO_LATENCY_TEST
    LEDFrame = DetectLEDFrame(Dat, Len);
    
    if(true == LEDFrame)
    {
        FrameCTime = 0;
        FrameTimes[FrameCTime ++] = GetNowUs();
        SetLED(0, true);
    }
    #endif
    GetOneAllocInputBuffer(c->pVideoEnc, &(c->inputBuffer));
    
    c->inputBuffer.bEnableCorp = 0;
    c->inputBuffer.sCropInfo.nLeft =  0;
    c->inputBuffer.sCropInfo.nTop  =  0;
    c->inputBuffer.sCropInfo.nWidth  =  0;
    c->inputBuffer.sCropInfo.nHeight =  0;
    FlushCacheAllocInputBuffer(c->pVideoEnc, &c->inputBuffer);
    c->pts += 1000/c->h264Param.nFramerate;
    c->inputBuffer.nPts = c->pts;
    AddOneInputBuffer(c->pVideoEnc, &c->inputBuffer);
    
    int DatLen =  (Len / 3);
 //   fprintf(stderr, "DataLen = %d, w x h =%d, Copylen=%d\n", Len, 1280*720, DatLen);
	memcpy(c->inputBuffer.pAddrVirY, Dat, DatLen * 2);
	memcpy(c->inputBuffer.pAddrVirC, Dat + (DatLen * 2), DatLen);
    #ifdef DO_LATENCY_TEST
    if(true == LEDFrame)
    {
        FrameTimes[FrameCTime ++] = GetNowUs();
    }
    #endif
}

void h264enc_done_outputbuffer(h264enc *c)
{
    FreeOneBitStreamFrame(c->pVideoEnc, &c->outputBuffer);
    #ifdef DO_LATENCY_TEST
    if(true == LEDFrame)
    {
        FrameTimes[FrameCTime ++] = GetNowUs();
        fprintf(stderr, "Frame time: Start[%d], Inp=%d, Enc=%d, Tot=%d\n", (unsigned int) FrameTimes[0], (unsigned int)(FrameTimes[1]-FrameTimes[0]), (unsigned int)(FrameTimes[3]-FrameTimes[2]), (unsigned int)(FrameTimes[4]-FrameTimes[0]));
        LEDFrame = false;
        SetLED(0, false);
        SetLED(1, false);
    }
    #endif
}

void *h264enc_get_bytestream_buffer(const h264enc *c, int stream)
{
    PMSG("h264enc_get_bytestream_buffer()");
    if(stream == 0)
    {
        return c->outputBuffer.pData0;
    }
    else
    {
        return c->outputBuffer.pData1;
    }
}

unsigned int h264enc_is_keyframe(const h264enc *c)
{
    if(c->outputBuffer.nFlag & VENC_BUFFERFLAG_KEYFRAME)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

unsigned int h264enc_get_bytestream_length(const h264enc *c, int stream)
{
    //fprintf(stderr, "Pete: h264enc_get_bytestream_length()=%d (%d)\n", c->outputBuffer.nSize0, c->outputBuffer.nSize1);
    if(stream == 0)
    {
        return c->outputBuffer.nSize0;
    }
    else
    {
        return c->outputBuffer.nSize1;
    }
}

int h264enc_encode_picture(h264enc *c)
{
    int result;
    #ifdef DO_LATENCY_TEST
    if(true == LEDFrame)
    {
        FrameTimes[FrameCTime ++] = GetNowUs();
    }
    #endif
    PMSG("h264enc_encode_picture()");
    VideoEncodeOneFrame(c->pVideoEnc);
    
    AlreadyUsedInputBuffer(c->pVideoEnc,&c->inputBuffer);
    ReturnOneAllocInputBuffer(c->pVideoEnc, &c->inputBuffer);

    result = GetOneBitstreamFrame(c->pVideoEnc, &c->outputBuffer);
    if(result == -1)
    {
        printf("h264enc_encode_picture() Could not get result buffer\n");
    }
    PMSG("h264enc_encode_picture() complete");
    #ifdef DO_LATENCY_TEST
    if(true == LEDFrame)
    {
        FrameTimes[FrameCTime ++] = GetNowUs();
    }
    #endif
	return 1;
}
