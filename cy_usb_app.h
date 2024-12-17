/***************************************************************************//**
* \file cy_usb_app.h
* \version 1.0
*
* Defines the interfaces used in the FX10/FX20 USB Video Class application.
*
*******************************************************************************
* \copyright
* (c) (2021-2023), Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.
*
* SPDX-License-Identifier: Apache-2.0
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#ifndef _CY_USB_APP_H_
#define _CY_USB_APP_H_

#include "cy_pdl.h"
#include "cy_debug.h"
#include "cy_usbhs_dw_wrapper.h"

#if defined(__cplusplus)
extern "C" {
#endif

#define RED                             "\033[0;31m"
#define CYAN                            "\033[0;36m"
#define COLOR_RESET                     "\033[0m"

#define LOG_COLOR(...)                  Cy_Debug_AddToLog(1,CYAN);\
                                        Cy_Debug_AddToLog(1,__VA_ARGS__); \
                                        Cy_Debug_AddToLog(1,COLOR_RESET);

#define LOG_ERROR(...)                  Cy_Debug_AddToLog(1,RED);\
                                        Cy_Debug_AddToLog(1,__VA_ARGS__); \
                                        Cy_Debug_AddToLog(1,COLOR_RESET);

#define LOG_CLR(CLR, ...)               Cy_Debug_AddToLog(1,CLR);\
                                        Cy_Debug_AddToLog(1,__VA_ARGS__); \
                                        Cy_Debug_AddToLog(1,COLOR_RESET);


#define LOG_TRACE()                     LOG_COLOR("-->[%s]:%d\r\n",__func__,__LINE__);


#define DELAY_MICRO(us)                 Cy_SysLib_DelayUs(us)
#define DELAY_MILLI(ms)                 Cy_SysLib_Delay(ms)

#define PHY_TRAINING_PATTERN_BYTE       (0x39)
#define LINK_TRAINING_PATTERN_BYTE      (0xAA55AA55) // Working all the time with 100u phy train interval
#define FPS_DEFAULT                     (60)

#define    SET_BIT(byte, mask)                (byte) |= (mask)
#define CLR_BIT(byte, mask)                (byte) &= ~(mask)
#define CHK_BIT(byte, mask)                (byte) & (mask)


#if !LVDS_LB_EN

#if FPGA_ADDS_HEADER
#define UVC_HEADER_BY_FPGA              (1)
#define UVC_HEADER_BY_FX10              (0)

/* DMA channel and UVC header configuration*/
#define MANUAL_DMA_CHANNEL              (0)
#define AUTO_DMA_CHANNEL                (!MANUAL_DMA_CHANNEL)
#else
#define UVC_HEADER_BY_FPGA              (0)
#define UVC_HEADER_BY_FX10              (1)
/* Do not change thes value for fx10 added header*/
#define MANUAL_DMA_CHANNEL              (1)
#endif /* FPGA_ADDS_HEADER */

/* Choose video source */
#define VIDEO_SOURCE
#define SOURCE_COLORBAR                 (1)

#if (LVCMOS_EN && (!LVCMOS_DDR_EN))
#define LINK_TRAINING                   (0)
#else
#define LINK_TRAINING                   (1)
#endif /* LVCMOS_EN && (!LVCMOS_DDR_EN) */

#define LINK_READY_CTL_PIN              (6)

#else 

#define MANUAL_DMA_CHANNEL              (1)
#define UVC_HEADER_BY_FX10              (1)
#define UVC_HEADER_BY_FPGA              (AUTO_DMA_CHANNEL || (!UVC_HEADER_BY_FX10))
#endif /*LVDS_LB_EN*/

#if LVDS_LB_EN && FPGA_ENABLE
#error LOG_COLOR(RED, "INVALID: LVDS_LB_EN WITH FPGA_ENABLE");
#endif /* LVDS_LB_EN && FPGA_ENABLE */

#if UVC_HEADER_BY_FPGA && LVDS_LB_EN
#error LOG_COLOR(RED, "INVALID: LOOPBACK WITH UVC_HEADER_BY_FPGA");
#endif

#if PORT1_EN && INTERLEAVE_EN
#error LOG_COLOR(RED, "INVALID: INTERLEAVE NOT SUPPORTED BY PORT1");
#endif

#define DMA_BUFFER_SIZE                 (CY_USB_UVC_STREAM_BUF_SIZE)
#if UVC_HEADER_BY_FX10
#define FPGA_DMA_BUFFER_SIZE            (DMA_BUFFER_SIZE - CY_USB_UVC_MAX_HEADER)
#elif UVC_HEADER_BY_FPGA
#define FPGA_DMA_BUFFER_SIZE            (DMA_BUFFER_SIZE)
#endif

/* GPIO port pins*/
#define TI180_CRESET_GPIO                (P4_3_GPIO)
#define TI180_CRESET_GPIO_PORT          (P4_3_PORT)
#define TI180_CRESET_GPIO_PIN           (P4_3_PIN)


#define ASSERT(condition, value)        Cy_CheckStatus(__func__, __LINE__, condition, value, true);
#define ASSERT_NON_BLOCK(condition, value) Cy_CheckStatus(__func__, __LINE__, condition, value, false);
#define ASSERT_AND_HANDLE(condition, value, failureHandler) Cy_Cy_CheckStatusAndHandleFailure(__func__, __LINE__, condition, value, false, failureHandler);

/* Loopback program color bands*/
#define BAND1_COLOR_YUYV                0x80ff80ff    // White
#define BAND2_COLOR_YUYV                0x94ff00ff    // Yellow
#define BAND3_COLOR_YUYV                0x1ac8bfc8    // Blue
#define BAND4_COLOR_YUYV                0x4aca55ca    // Green
#define BAND5_COLOR_YUYV                0xf3969f96    // Pink
#define BAND6_COLOR_YUYV                0xff4c544c    // Red
#define BAND7_COLOR_YUYV                0x9e40d340    // Violet
#define BAND8_COLOR_YUYV                0x80008000    // Black
#define COLORBAR_BAND_COUNT_4K          120
#define COLORBAR_BAND_COUNT_1080P       60
#define COLORBAR_BAND_COUNT_720P        40
#define COLORBAR_BAND_COUNT_480P        20
#define LOOPBACK_MEM_BUF_SIZE           0xF0C0

/* P4.0 is used for VBus detect functionality. */
#define VBUS_DETECT_GPIO_PORT           (P4_0_PORT)
#define VBUS_DETECT_GPIO_PIN            (P4_0_PIN)
#define VBUS_DETECT_GPIO_INTR           (ioss_interrupts_gpio_dpslp_4_IRQn)
#define VBUS_DETECT_STATE               (0u)

/* Number of buffers used for PDM data transfer. */
#define PDM_APP_BUFFER_CNT              (4u)

#define LVCMOS_GPIF_CTRLBUS_BITMAP      (0x0000038F)
#define LVCMOS_GPIF_CTRLBUS_BITMAP_WL   (0x000C008F)

typedef struct cy_stc_usb_app_ctxt_ cy_stc_usb_app_ctxt_t;

/* USBD layer return code shared between USBD layer and Application layer. */
typedef enum cy_en_usb_app_ret_code_ {
    CY_USB_APP_STATUS_SUCCESS=0,
    CY_USB_APP_STATUS_FAILURE,
}cy_en_usb_app_ret_code_t;

/*
 * USB application data structure which is bridge between USB system and device
 * functionality.
 * It maintains some usb system information which comes from USBD and it also
 * maintains info about functionality.
 */
struct cy_stc_usb_app_ctxt_
{
    uint8_t firstInitDone;
    cy_en_usb_device_state_t devState;
    cy_en_usb_device_state_t prevDevState;
    cy_en_usb_speed_t devSpeed;
    uint8_t devAddr;
    uint8_t activeCfgNum;
    cy_en_usb_enum_method_t enumMethod;
    uint8_t prevAltSetting;
    cy_en_usb_speed_t desiredSpeed;

    cy_stc_app_endp_dma_set_t endpInDma[CY_USB_MAX_ENDP_NUMBER];
    cy_stc_app_endp_dma_set_t endpOutDma[CY_USB_MAX_ENDP_NUMBER];
    DMAC_Type *pCpuDmacBase;
    DW_Type *pCpuDw0Base;
    DW_Type *pCpuDw1Base;

    cy_stc_hbdma_mgr_context_t *pHbDmaMgrCtxt;
    cy_stc_usb_usbd_ctxt_t *pUsbdCtxt;
    cy_stc_hbdma_channel_t *hbBulkInChannel;

    /* UVC specific fields. */
    uint8_t uvcInEpNum;                         /** Index of UVC streaming endpoint. */
    uint8_t uvcPendingBufCnt;                   /** Number of data buffers which are pending to be streamed. */
    bool uvcFlowCtrlFlag;                       /** Flag indicating that UVC channel is in flow control. */

    /* Global Task handles */
    TaskHandle_t uvcDevicetaskHandle;           /** UVC application task. */
    QueueHandle_t uvcMsgQueue;                  /** Message queue used to send messages to the UVC task. */

    bool isLpmEnabled;                          /** Whether LPM transitions are enabled. */
    uint32_t lpmEnableTime;                     /** Timestamp at which LPM should be re-enabled. */

    bool usbConnectDone;
    bool vbusChangeIntr;                        /** VBus change interrupt received flag. */
    bool vbusPresent;                           /** VBus presence indicator flag. */
    bool usbConnected;                          /** Whether USB connection is enabled. */
    TimerHandle_t vbusDebounceTimer;            /** VBus change debounce timer handle. */
    uint32_t *pUsbEvtLogBuf;
    TimerHandle_t evtLogTimer;                  /** Timer to print eventLog. */

#if AUDIO_IF_EN
    /* UAC interface specific fields. */
    cy_stc_hbdma_channel_t *pPDMToUsbChn;       /** PDM to USB DMA channel handle. */
    uint8_t *pPDMRxBuffer[PDM_APP_BUFFER_CNT];  /** Pointer of buffers to read PDM data into. */
    uint16_t pdmRxDataLen[PDM_APP_BUFFER_CNT];  /** Amount of data in each PDM RX buffer. */
    uint8_t  pdmRxBufIndex;                     /** Index of current PDM read buffer. */
    uint8_t  nxtAudioTxBufIndex;                /** Index of next audio buffer to be sent on USB EP. */
    uint8_t  pdmRxFreeBufCount;                 /** Number of free PDM RX buffers. */
    bool     pdmInXferPending;                  /** Whether a write has been queued on IN EP. */
    uint8_t  pdmPendingDmaFlag;                 /** Flag indicating whether DMA transfer is pending on
                                                    each PDM RX channel. */
    TaskHandle_t  uacAppTaskHandle;             /** Handle to the UAC application task. */
    QueueHandle_t uacMsgQueue;                  /** Handle to UAC application message queue. */
    TimerHandle_t pdmActivateTimer;             /** Handle of timer used to activate PDM channels. */ 
#endif /* AUDIO_IF_EN */

    TimerHandle_t fpsTimer;
    uint8_t fpgaVersion;
    uint8_t glpassiveSerialMode;
    uint8_t *qspiWriteBuffer;
    uint8_t *qspiReadBuffer;

    uint32_t glfps;
    uint32_t glDmaBufCnt;
    uint32_t glDmaBufCnt_prv;
    volatile uint32_t glProd;
    volatile uint32_t glCons;
    uint32_t glProdCount;
    uint32_t glConsCount;
    uint32_t glFrameSizeTransferred;
    uint32_t glFrameSize;
    volatile uint8_t glPrintFlag;
    volatile uint32_t glFrameCount;
    volatile uint32_t glPartialBufSize;
};

typedef struct
{
    uint8_t *pBuffer;
    uint8_t start;
    uint8_t end;
    uint8_t dataMode;
    uint8_t dataSrc;
    uint8_t ctrlByte;
    uint16_t repeatCount;
    uint32_t dataL;
    uint32_t dataH;
    uint32_t ctrlBusVal;
    uint32_t lbPgmCount;
} cy_stc_lvds_loopback_config_t;

typedef struct
{
    uint32_t dataWord0;
    uint32_t dataWord1;
    uint32_t dataWord2;
    uint32_t dataWord3;
} cy_stc_lvds_loopback_mem_t;

typedef enum cy_en_fpgaRegMap_t
{
    /*Common Register Info*/
    FPGA_MAJOR_VERSION_ADDRESS             = 0x00,
    FPGA_MINOR_VERSION_ADDRESS             = 0x00,

    FPGA_UVC_U3V_SELECTION_ADDRESS         = 0x01,
    FPGA_UVC_ENABLE                        = 1,
    FPGA_U3V_ENABLE                        = 0,

    FPGA_UVC_HEADER_CTRL_ADDRESS           = 0x02,
    FPGA_UVC_HEADER_ENABLE                 = 1,
    FPGA_UVC_HEADER_DISABLE                = 0, // FX10 will add UVC Header

    FPGA_LVDS_PHY_TRAINING_ADDRESS         = 0x03,
    FPGA_LVDS_PHY_TRAINING_DATA            = 0x39,

    FPGA_LVDS_LINK_TRAINING_BLK_P0_ADDRESS = 0x04,
    FPGA_LVDS_LINK_TRAINING_BLK_P1_ADDRESS = 0x05,
    FPGA_LVDS_LINK_TRAINING_BLK_P2_ADDRESS = 0x06,
    FPGA_LVDS_LINK_TRAINING_BLK_P3_ADDRESS = 0x07,
    
    FPGA_ACTIVE_DIVICE_MASK_ADDRESS        = 0x08,
    FPGA_LOW_PWR_MODE_ADDRESS              = 0x09,
    
    FPGA_PHY_LINK_CONTROL_ADDRESS          = 0x0A,
    FPGA_TRAINING_DISABLE                  = 0x00,
    FPGA_PHY_CONTROL                       = 0x01, // PHY Training is required
    FPGA_LINK_CONTROL                      = 0x02, //  Link Training is required

    P0_TRAINING_DONE                       = 0X40, // Port 0 training is completed
    P1_TRAINING_DONE                       = 0X80, // Port 1 training is completed

    FPGA_EXT_CONTROLLER_STATUS_ADDRESS     = 0X0B,
    DMA_READY_STATUS                       = 0x01, // DMA Ready flag status
    DDR_CONFIG_STATUS                      = 0x02, // DDR configuration status
    DDR_BUSY_STATUS                        = 0x04, // DDR Controller busy status
    DDR_CMD_QUEUE_FULL_STATUS              = 0x08, // Command queue full status
    DATPATH_IDLE_STATUS                    = 0x10, // Datapath is idle or not


    /*Device related Info*/
    DEVICE0_OFFSET                         = 0x20,
    DEVICE1_OFFSET                         = 0x3C,
    DEVICE2_OFFSET                         = 0x58,
    DEVICE3_OFFSET                         = 0x74,
    DEVICE4_OFFSET                         = 0x90,
    DEVICE5_OFFSET                         = 0xAC,
    DEVICE6_OFFSET                         = 0xC8,
    DEVICE7_OFFSET                         = 0xE4,

    FPGA_DEVICE_STREAM_ENABLE_ADDRESS      = 0x00,
    CAMERA_APP_DISABLE                     = 0x00,
    DMA_CH_RESET                           = 0x01,
    CAMERA_APP_ENABLE                      = 0x02,
    APP_STOP_NOTIFICATION                  = 0x04,

    FPGA_DEVICE_STREAM_MODE_ADDRESS        = 0x01,
    NO_CONVERSION                          = 0,
    INTERLEAVED_MODE                       = 0x01,
    STILL_CAPTURE                          = 0x02,
    MONO_8_CONVERSION                      = 0x04,
    YUV422_420_CONVERSION                  = 0x08,

    DEVICE_IMAGE_HEIGHT_LSB_ADDRESS        = 0x02,
    DEVICE_IMAGE_HEIGHT_MSB_ADDRESS        = 0x03,
    DEVICE_IMAGE_WIDTH_LSB_ADDRESS         = 0x04,
    DEVICE_IMAGE_WIDTH_MSB_ADDRESS         = 0x05,
    
    DEVICE_FPS_ADDRESS                     = 0x06,

    DEVICE_PIXEL_WIDTH_ADDRESS             = 0x07,
    _8_BIT_PIXEL                           = 8,
    _12BIT_PIXEL                           = 12,
    _16BIT_PIXEL                           = 16,
    _24BIT_PIXEL                           = 24,
    _36BIT_PIXEL                           = 36,

    DEVICE_SOURCE_TYPE_ADDRESS             = 0x08,
    INTERNAL_COLORBAR                      = 0x00,
    HDMI_SOURCE                            = 0x01,
    MIPI_SOURCE                            = 0x02,

    DEVICE_FLAG_STATUS_ADDRESS             = 0x09,
    SLAVE_FIFO_ALMOST_EMPTY                = 0x01, // Slave FIFO almost empty status
    INTER_MED_FIFO_EMPTY                   = 0x02, // Intermediate FIFO empty status
    INTER_MED_FIFO_FULL                    = 0x04, // Intermediate FIFO full status
    DDR_FULL_FRAME_WRITE_COMPLETE          = 0x10, // DDR write status (Full frame write complete)
    DDR_FULL_FRAME_READ_COMPLETE           = 0x20, // DDR write status (Full frame read complete)

    DEVICE_MIPI_STATUS_ADDRESS             = 0x0A,

    DEVICE_HDMI_SOURCE_INFO_ADDRESS        = 0x0B,
    HDMI_DISCONECT                         = 0x00, // Source connection status
    THIN_MIPI                              = 0x00,
    HDMI_CONNECT                           = 0x01, // Source connection status
    HDMI_DUAL_CH                           = 0x02, // 0 for single channel and 1 for dual channel
    SONY_CIS                               = 0x10, // Enable CIS ISP IP (Valid for only MIPI source)
    SONY_MIPI                              = 0x20, // Enable Crop Algorithm (Valid for only MIPI source)

    DEVICE_U3V_STREAM_MODE_ADDRESS         = 0x0C,
    DEVICE_U3V_CHUNK_MODE_ADDRESS          = 0x0D,
    DEVICE_U3V_TRIGGER_MODE_ADDRESS        = 0x0E,
    DEVICE_ACTIVE_TREAD_INFO_ADDRESS       = 0x0F,
    DEVICE_THREAD1_INFO_ADDRESS            = 0x10,
    DEVICE_THREAD2_INFO_ADDRESS            = 0x11,
    DEVICE_THREAD1_SOCKET_INFO_ADDRESS     = 0x12,
    DEVICE_THREAD2_SOCKET_INFO_ADDRESS     = 0x13,

    DEVICE_FLAG_INFO_ADDRESS               = 0x14,
    FX10_READY_TO_REC_DATA                 = 0x08,
    NEW_UVC_PACKET_START                   = 0x02,
    NEW_FRAME_START                        = 0x01,

    DEVICE_COUNTER_CRC_INFO_ADDRESS        = 0x15,
    DEVICE_BUFFER_SIZE_LSB_ADDRESS         = 0x16,
    DEVICE_BUFFER_SIZE_MSB_ADDRESS         = 0x17,

} cy_en_fpgaRegMap_t;

/* FPGA Configuration mode selection*/
typedef enum cy_en_fpgaConfigMode_t
{
    ACTIVE_SERIAL_MODE,
    PASSIVE_SERIAL_MODE
}cy_en_fpgaConfigMode_t;

typedef enum cy_en_streamControl_t
{
    STOP,
    START
}cy_en_streamControl_t;

extern cy_stc_hbdma_channel_t lvdsLbPgmChannel;
extern uint8_t glPhyLinkTrainControl;

void Cy_LVDS_InitLbPgm(cy_stc_hbdma_buff_status_t *buffStat,
                       cy_stc_lvds_loopback_config_t *lbPgmConfig);
void Cy_HbDma_LoopbackCb(cy_stc_hbdma_channel_t *handle,
                       cy_en_hbdma_cb_type_t type,
                       cy_stc_hbdma_buff_status_t *pbufStat,
                       void *userCtx);

void Cy_USB_AppInit(cy_stc_usb_app_ctxt_t *pAppCtxt,
                    cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                    DMAC_Type *pCpuDmacBase,
                    DW_Type *pCpuDw0Base,
                    DW_Type *pCpuDw1Base,
                    cy_stc_hbdma_mgr_context_t *pHbDmaMgrCtxt);

void Cy_USB_AppRegisterCallback(cy_stc_usb_app_ctxt_t *pAppCtxt);

uint32_t *Cy_USB_CalculateEpmAddr(uint32_t endpNum,
        cy_en_usb_endp_dir_t endpDirection);

void Cy_USB_AppSetCfgCallback(void *pAppCtxt,
                              cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                              cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppBusResetCallback(void *pAppCtxt,
                                cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppBusResetDoneCallback(void *pAppCtxt,
                                    cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                    cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppBusSpeedCallback(void *pAppCtxt,
                                cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppSetupCallback(void *pAppCtxt,
                             cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                             cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppSuspendCallback(void *pAppCtxt,
                               cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                               cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppResumeCallback (void *pAppCtxt,
                               cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                               cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppSetIntfCallback(void *pAppCtxt,
                               cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                               cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppL1SleepCallback(void *pUsbApp,
                               cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                               cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppL1ResumeCallback(void *pUsbApp,
                                cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppZlpCallback(void *pUsbApp, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                           cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppSlpCallback(void *pUsbApp, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                           cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppSetFeatureCallback(void *pUsbApp,
                                  cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                  cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppClearFeatureCallback(void *pUsbApp,
                                  cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                  cy_stc_usb_cal_msg_t *pMsg);
/* Function to register all USB descriptors with the stack. */
void CyApp_RegisterUsbDescriptors(cy_stc_usb_app_ctxt_t *pAppCtxt,
                                    cy_en_usb_speed_t usbSpeed);
void Cy_USB_AppQueueRead (cy_stc_usb_app_ctxt_t *pAppCtxt,
        uint8_t endpNumber, uint8_t *pBuffer, uint16_t dataSize);

uint16_t Cy_USB_AppReadShortPacket(cy_stc_usb_app_ctxt_t *pAppCtxt,
        uint8_t endpNumber, uint16_t pktSize);

void Cy_USB_AppQueueWrite (cy_stc_usb_app_ctxt_t *pAppCtxt,
        uint8_t endpNumber, uint8_t *pBuffer, uint16_t dataSize);

void Cy_USB_AppInitDmaIntr(uint32_t endpNumber,
        cy_en_usb_endp_dir_t endpDirection,
        cy_israddress userIsr);

void Cy_USB_AppClearDmaInterrupt(cy_stc_usb_app_ctxt_t *pAppCtxt,
        uint32_t endpNumber, cy_en_usb_endp_dir_t endpDirection);

void Cy_UVC_AppHandleSendCompletion(cy_stc_usb_app_ctxt_t *pUsbApp);

void Cy_UVC_DataWire_ISR(void);

/* Enable the USB data connection. */
bool Cy_USB_SSConnectionEnable(cy_stc_usb_app_ctxt_t *pAppCtxt);

/* Disable the USB data connection. */
void Cy_USB_SSConnectionDisable(cy_stc_usb_app_ctxt_t *pAppCtxt);

/* Initialize the LVDS/LVCMOS interface to receive the video data through. */
void Cy_LVDS_LVCMOS_Init(void);

/* Configure USB endpoints based on the settings provided in the endpoint descriptor. */
void Cy_USB_AppConfigureEndp(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, uint8_t *pEndpDscr);

/* Configure application level DMA resources corresponding to a USB endpoint. */
void Cy_USB_AppSetupEndpDmaParams(cy_stc_usb_app_ctxt_t *pUsbApp, uint8_t *pEndpDscr);

/* Initialize the PDM IP to receive data from PDM microphone. */
void Cy_PDM_AppMxPdmInit(void);

/* De-initialize the PDM IP. */
void Cy_PDM_AppMxPdmDeinit(void);

/* Enable or disable the interrupt handlers for the DataWire channels reading
 * data out from the PDM receive FIFOs.
 */
void Cy_UAC_AppInitDmaIntr(bool enable);

/* Interrupt service routine for completion of data transfer on USB 2.x
 * IN endpoint used for audio streaming.
 */
void Cy_PDM_InEpDma_ISR(void);

/* Initialize UAC application context structures. */
void Cy_UAC_AppInit(cy_stc_usb_app_ctxt_t *pAppCtxt);

/* Queue a read using DataWire channels from the PDM receive FIFO. */
void Cy_PDM_AppQueueRead(cy_stc_usb_app_ctxt_t *pAppCtxt, uint32_t dataLength);

/* Handler for SET_INTERFACE requests addressed to the UAC streaming interface. */
void Cy_UAC_AppSetIntfHandler(cy_stc_usb_app_ctxt_t *pAppCtxt, uint8_t altSetting);

/* Function to print any new USB event log data since the last call. */
void Cy_USB_AppPrintUsbEventLog(cy_stc_usb_app_ctxt_t *pAppCtxt, cy_stc_usbss_cal_ctxt_t *pSSCal);

void Cy_CheckStatus(const char *function, uint32_t line, uint8_t condition, uint32_t value, uint8_t isBlocking);

void SendEndp0DataFailHandler(void);

void Cy_CheckStatusAndHandleFailure(const char *function, uint32_t line, uint8_t condition, uint32_t value, uint8_t isBlocking, void (*failureHandler)());

void Cy_SetCtlPinValue(bool port, uint8_t ctlPin, bool value);

/*Send Link Traning info to FPGA*/
cy_en_scb_i2c_status_t Cy_FPGAPhyLnkTraining (void);

#if defined(__cplusplus)
}
#endif

#endif /* _CY_USB_APP_H_ */

/* End of File */

