/*
 * Copyright 2016-2020 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
/**
 * @file    MIMXRT1062xxxxA_HelloWorld.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MIMXRT1062.h"
#include "fsl_debug_console.h"
/* TODO: insert other include files here. */
#include "fsl_os_abstraction.h"
#include "fsl_gpio.h"
#include "fsl_iomuxc.h"
#include "fsl_sd_disk.h"

/* TODO: insert other definitions and declarations here. */
#define LED_GPIO		GPIO2
#define LED_GPIO_PIN	3U

#define SDMMC_HOST_DMA_DESCRIPTOR_BUFFER_SIZE (32U)

void OSA_TimeInit(void);

static FATFS FatFs_Demo;
static FIL   FatFs_FileObject;   /* File object */
static FRESULT FatFs_Result;

/*
 * SD Card initialization using the SDHC module
 */
void Teensy_SDCardInserted(bool isInserted, void *userData) {
	PRINTF("Teensy_SDCardInserted: %d\n", isInserted);
}

AT_NONCACHEABLE_SECTION_ALIGN(static uint32_t s_sdmmcHostDmaBuffer[SDMMC_HOST_DMA_DESCRIPTOR_BUFFER_SIZE], 4U);
static sd_detect_card_t s_sdDetectCard = {
    .type          = kSD_DetectCardByHostDATA3,
    .cdDebounce_ms = 100U,
    .callback      = Teensy_SDCardInserted,
    .cardDetected  = NULL,
    .userData      = NULL,
};
static sd_io_voltage_t s_sdIoVoltage = {
    .type = kSD_IOVoltageCtrlNotSupport,
    .func = (sd_io_voltage_func_t)NULL,
};
static sdmmchost_t s_host;
OSA_EVENT_HANDLE_DEFINE(s_event);


uint32_t Teensy_USDHC1ClockConfiguration(void)
{
//    CLOCK_InitSysPll(&sysPllConfig_BOARD_BootClockRUN);
//    /*configure system pll PFD0 fractional divider to 24, output clock is 528MHZ * 18 / 24 = 396 MHZ*/
//    CLOCK_InitSysPfd(kCLOCK_Pfd0, 24U);
//    /* Configure USDHC clock source and divider */
//    CLOCK_SetDiv(kCLOCK_Usdhc1Div, 1U); /* USDHC clock root frequency maximum: 198MHZ */
//    CLOCK_SetMux(kCLOCK_Usdhc1Mux, 1U);

    return 396000000U / 2U;
}

void Teensy_SDPinConfig(uint32_t freq)
{
    uint32_t speed = 0U, strength = 0U;

    if (freq <= 50000000)
    {
        speed    = 0U;
        strength = 7U;
    }
    else if (freq <= 100000000)
    {
        speed    = 2U;
        strength = 7U;
    }
    else
    {
        speed    = 3U;
        strength = 7U;
    }

    IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B0_00_USDHC1_CMD,
                        IOMUXC_SW_PAD_CTL_PAD_SPEED(speed) | IOMUXC_SW_PAD_CTL_PAD_SRE_MASK |
                            IOMUXC_SW_PAD_CTL_PAD_PKE_MASK | IOMUXC_SW_PAD_CTL_PAD_PUE_MASK |
                            IOMUXC_SW_PAD_CTL_PAD_HYS_MASK | IOMUXC_SW_PAD_CTL_PAD_PUS(1) |
                            IOMUXC_SW_PAD_CTL_PAD_DSE(strength));
    IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B0_01_USDHC1_CLK,
                        IOMUXC_SW_PAD_CTL_PAD_SPEED(speed) | IOMUXC_SW_PAD_CTL_PAD_SRE_MASK |
                            IOMUXC_SW_PAD_CTL_PAD_HYS_MASK | IOMUXC_SW_PAD_CTL_PAD_PUS(0) |
                            IOMUXC_SW_PAD_CTL_PAD_DSE(strength));
    IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B0_02_USDHC1_DATA0,
                        IOMUXC_SW_PAD_CTL_PAD_SPEED(speed) | IOMUXC_SW_PAD_CTL_PAD_SRE_MASK |
                            IOMUXC_SW_PAD_CTL_PAD_PKE_MASK | IOMUXC_SW_PAD_CTL_PAD_PUE_MASK |
                            IOMUXC_SW_PAD_CTL_PAD_HYS_MASK | IOMUXC_SW_PAD_CTL_PAD_PUS(1) |
                            IOMUXC_SW_PAD_CTL_PAD_DSE(strength));
    IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B0_03_USDHC1_DATA1,
                        IOMUXC_SW_PAD_CTL_PAD_SPEED(speed) | IOMUXC_SW_PAD_CTL_PAD_SRE_MASK |
                            IOMUXC_SW_PAD_CTL_PAD_PKE_MASK | IOMUXC_SW_PAD_CTL_PAD_PUE_MASK |
                            IOMUXC_SW_PAD_CTL_PAD_HYS_MASK | IOMUXC_SW_PAD_CTL_PAD_PUS(1) |
                            IOMUXC_SW_PAD_CTL_PAD_DSE(strength));
    IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B0_04_USDHC1_DATA2,
                        IOMUXC_SW_PAD_CTL_PAD_SPEED(speed) | IOMUXC_SW_PAD_CTL_PAD_SRE_MASK |
                            IOMUXC_SW_PAD_CTL_PAD_PKE_MASK | IOMUXC_SW_PAD_CTL_PAD_PUE_MASK |
                            IOMUXC_SW_PAD_CTL_PAD_HYS_MASK | IOMUXC_SW_PAD_CTL_PAD_PUS(1) |
                            IOMUXC_SW_PAD_CTL_PAD_DSE(strength));
    IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B0_05_USDHC1_DATA3,
                        IOMUXC_SW_PAD_CTL_PAD_SPEED(speed) | IOMUXC_SW_PAD_CTL_PAD_SRE_MASK |
                            IOMUXC_SW_PAD_CTL_PAD_PKE_MASK | IOMUXC_SW_PAD_CTL_PAD_PUE_MASK |
                            IOMUXC_SW_PAD_CTL_PAD_HYS_MASK | IOMUXC_SW_PAD_CTL_PAD_PUS(1) |
                            IOMUXC_SW_PAD_CTL_PAD_DSE(strength));
}

void Teensy_SDConfig(void *card, sd_cd_t cd, uint32_t hostIRQPriority, void *userData)
{
    assert(card);

    s_host.dmaDesBuffer                                      = s_sdmmcHostDmaBuffer;
    s_host.dmaDesBufferWordsNum                              = SDMMC_HOST_DMA_DESCRIPTOR_BUFFER_SIZE;
    ((sd_card_t *)card)->host                                = &s_host;
    ((sd_card_t *)card)->host->hostController.base           = USDHC1;
    ((sd_card_t *)card)->host->hostController.sourceClock_Hz = Teensy_USDHC1ClockConfiguration();

    ((sd_card_t *)card)->host->hostEvent     = &s_event;
    ((sd_card_t *)card)->usrParam.cd         = &s_sdDetectCard;
    ((sd_card_t *)card)->usrParam.pwr        = NULL;
    ((sd_card_t *)card)->usrParam.ioStrength = Teensy_SDPinConfig;
    ((sd_card_t *)card)->usrParam.ioVoltage  = &s_sdIoVoltage;
    ((sd_card_t *)card)->usrParam.maxFreq    = 200000000U;

    NVIC_SetPriority(USDHC1_IRQn, hostIRQPriority);
}

static status_t Teensy_WaitForCardToBeInserted(void)
{
    Teensy_SDConfig(&g_sd, NULL, 5U, NULL);

    PRINTF("Initializing SD Host\n");
    /* SD host init function */
    if (SD_HostInit(&g_sd) != kStatus_Success)
    {
        PRINTF("SD host init fail\n");
        return kStatus_Fail;
    }
    /* power off card */
    SD_SetCardPower(&g_sd, false);

    PRINTF("Waiting for card to be inserted...\n");
    /* wait card insert */
    uint32_t cardDetectCount = 0;
    status_t cardDetectStatus;
    while ((cardDetectCount < 120)
    		&& ((cardDetectStatus = SD_PollingCardInsert(&g_sd, kSD_Inserted)) == kStatus_Fail))
    {
        PRINTF("Card detect fail.  waiting...\n");
        OSA_TimeDelay(1000);
    }
    if (cardDetectStatus == kStatus_Success) {
        PRINTF("\nCard inserted.\n");
        /* power on the card */
        SD_SetCardPower(&g_sd, true);
    } else {
        return kStatus_Fail;
    }

    return kStatus_Success;
}

/*
 * @brief   Application entry point.
 */
int main(void) {
  	/* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
    OSA_TimeInit();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();
#endif

    printf("After BOARD initialization...\n");

    PRINTF("Initializing SD Card Host...\n");
    status_t sd_Result = Teensy_WaitForCardToBeInserted();
    PRINTF("Teensy_InitSdHost: [%d]\n", sd_Result);

    PRINTF("Mounting SD filesystem...\n");
    FatFs_Result = f_mount(&FatFs_Demo, (const TCHAR*)"2:", 1);
    PRINTF("Mount: [%d]\n", FatFs_Result);

    printf("Running...\n");
    PRINTF("Hello World\n");

    PRINTF("Create directory...\n");
    FatFs_Result = f_mkdir("2:/demo");
    if (FatFs_Result) {
    	PRINTF("Create directory failed: [%d]\n", FatFs_Result);
    }

    PRINTF("Opening file...\n");
    FatFs_Result = f_open(&FatFs_FileObject, "2:/demo/demo.log", (FA_WRITE | FA_READ | FA_CREATE_ALWAYS));
    if (FatFs_Result) {
    	PRINTF("Opening file failed: [%d]\n", FatFs_Result);
    }

    /* Force the counter to be placed into memory. */
    volatile static int i = 0 ;

    /* Enter an infinite loop, just incrementing a counter. */
    char helloMessage[64];
    uint32_t startMillis = OSA_TimeGetMsec();
    while(i < (1024U * 1024U)) {

        i++ ;

        _sprintf(helloMessage, "Counter: %d\n", i);
        printf(helloMessage);						// stdout
        PRINTF(helloMessage);						// DebugConsole
        f_puts(helloMessage, &FatFs_FileObject);	// FatFs /demo/demo.log

        GPIO_PinWrite(LED_GPIO, LED_GPIO_PIN, 1);
        OSA_TimeDelay(400);
        GPIO_PinWrite(LED_GPIO, LED_GPIO_PIN, 0);
        OSA_TimeDelay(600);

    }

    PRINTF("Closing file...\n");
    FatFs_Result = f_close(&FatFs_FileObject);
    if (FatFs_Result) {
    	PRINTF("Closing file failed: [%d]\n", FatFs_Result);
    }
    uint32_t elapsedMillis = OSA_TimeGetMsec() - startMillis;
    PRINTF("Run Time: %d.%d\n", elapsedMillis / 1000U, elapsedMillis % 1000U);

    return 0 ;
}
