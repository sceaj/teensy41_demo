/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Peripherals v8.0
processor: MIMXRT1062xxxxA
package_id: MIMXRT1062DVJ6A
mcu_data: ksdk2_0
processor_version: 8.0.2
functionalGroups:
- name: BOARD_InitPeripherals
  UUID: 0a792920-2873-4846-a9b9-5664b8f18016
  called_from_default_init: true
  selectedCore: core0
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/

/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
component:
- type: 'system'
- type_id: 'system_54b53072540eeeb8f8e9343e71f28176'
- global_system_definitions:
  - user_definitions: ''
  - user_includes: ''
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

/***********************************************************************************************************************
 * Included files
 **********************************************************************************************************************/
#include "peripherals.h"

/***********************************************************************************************************************
 * BOARD_InitPeripherals functional group
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * FATFS initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'FATFS'
- type: 'fatfs'
- mode: 'general'
- custom_name_enabled: 'false'
- type_id: 'fatfs_2f85acf758668258920f70258052a088'
- functional_group: 'BOARD_InitPeripherals'
- config_sets:
  - init_config:
    - initConfig:
      - initPartitionsStr: 'false'
      - multiplePartitions:
        - 0:
          - Volume: '0'
          - Partition: 'autoDetect'
        - 1:
          - Volume: '0'
          - Partition: 'autoDetect'
      - enablePhysicalLayerInit: 'false'
      - diskConfig:
        - initFunctionID: 'FATFS_DiskInit'
      - initResultObject: 'true'
      - resultName: 'FATFS_Result'
      - fatfsObjects:
        - 0:
          - objID: 'FATFS_System_0'
          - diskMount: 'true'
          - mountPath: '0:'
          - mountInitOpt: 'false'
      - filObjects: []
      - filInfoObjects: []
      - dirObjects: []
  - ff_config:
    - revisionID: 'rev14_1'
    - MSDKadaptation: 'RAM_DISK_ENABLE'
    - functionConfig:
      - FF_FS_READONLY: 'false'
      - FF_FS_MINIMIZE: 'level1'
      - FF_USE_STRFUNC: 'enableWithoutConversion'
      - FF_USE_FIND: 'disableDirRead'
      - FF_USE_MKFS: 'true'
      - FF_USE_FASTSEEK: 'false'
      - FF_USE_EXPAND: 'false'
      - FF_USE_CHMOD: 'false'
      - FF_USE_LABEL: 'false'
      - FF_USE_FORWARD: 'false'
    - namespaceConfig:
      - FF_USE_LFN: 'enableLfnStatic'
      - FF_MAX_LFN: '255'
      - FF_LFN_BUF: 'LFNID'
      - FF_SFN_BUF: 'SFNID'
      - FF_LFN_UNICODE: 'UTF8'
      - FF_STRF_ENCODE: 'UTF8'
      - FF_CODE_PAGE: 'cpUS'
      - FF_FS_RPATH: 'enableRP2'
    - driveConfig:
      - FF_VOLUMES: '6'
      - FF_STR_VOLUME_ID: 'numericId'
      - volumes:
        - 0:
          - volumeStr: 'RAM'
        - 1:
          - volumeStr: 'NAND'
        - 2:
          - volumeStr: 'CF'
        - 3:
          - volumeStr: 'SD'
        - 4:
          - volumeStr: 'SD2'
        - 5:
          - volumeStr: 'USB'
      - FF_MULTI_PARTITION: 'false'
      - FF_MIN_SS: 'value512'
      - FF_MAX_SS: 'value512'
      - FF_LBA64: 'false'
      - FF_MIN_GPT: '0x100000000'
      - FF_USE_TRIM: 'false'
    - systemConfig:
      - FF_FS_TINY: 'false'
      - FF_FS_EXFAT: 'true'
      - FF_FS_NORTC: 'true'
      - FF_NORTC_MON: '1'
      - FF_NORTC_MDAY: '1'
      - FF_NORTC_YEAR: '2020'
      - FF_FS_NOFSINFO: ''
      - FF_FS_LOCK: '0'
      - FF_FS_REENTRANT: 'false'
      - FF_FS_TIMEOUT: '1000'
      - FF_SYNC_t: 'HANDLE'
      - includeOS: 'false'
      - headerFileName: 'somertos.h'
    - fatfs_codegenerator: []
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
/* FATFS System object */
FATFS FATFS_System_0;
/* FATFS Result object */
FRESULT FATFS_Result;

static void FATFS_init(void) {
  /* FATFS Filesystem work area initialization */
  FATFS_Result = f_mount(&FATFS_System_0, (const TCHAR*)"0:", 0);
  assert(FATFS_Result == FR_OK);
}

/***********************************************************************************************************************
 * LPUART1 initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'LPUART1'
- type: 'lpuart'
- mode: 'polling'
- custom_name_enabled: 'false'
- type_id: 'lpuart_54a65a580e3462acdbacefd5299e0cac'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'LPUART1'
- config_sets:
  - lpuartConfig_t:
    - lpuartConfig:
      - clockSource: 'LpuartClock'
      - lpuartSrcClkFreq: 'BOARD_BootClockRUN'
      - baudRate_Bps: '115200'
      - parityMode: 'kLPUART_ParityDisabled'
      - dataBitsCount: 'kLPUART_EightDataBits'
      - isMsb: 'false'
      - stopBitCount: 'kLPUART_OneStopBit'
      - txFifoWatermark: '0'
      - rxFifoWatermark: '1'
      - enableRxRTS: 'false'
      - enableTxCTS: 'false'
      - txCtsSource: 'kLPUART_CtsSourcePin'
      - txCtsConfig: 'kLPUART_CtsSampleAtStart'
      - rxIdleType: 'kLPUART_IdleTypeStartBit'
      - rxIdleConfig: 'kLPUART_IdleCharacter1'
      - enableTx: 'true'
      - enableRx: 'true'
    - quick_selection: 'QuickSelection1'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const lpuart_config_t LPUART1_config = {
  .baudRate_Bps = 115200,
  .parityMode = kLPUART_ParityDisabled,
  .dataBitsCount = kLPUART_EightDataBits,
  .isMsb = false,
  .stopBitCount = kLPUART_OneStopBit,
  .txFifoWatermark = 0,
  .rxFifoWatermark = 1,
  .enableRxRTS = false,
  .enableTxCTS = false,
  .txCtsSource = kLPUART_CtsSourcePin,
  .txCtsConfig = kLPUART_CtsSampleAtStart,
  .rxIdleType = kLPUART_IdleTypeStartBit,
  .rxIdleConfig = kLPUART_IdleCharacter1,
  .enableTx = true,
  .enableRx = true
};

static void LPUART1_init(void) {
  LPUART_Init(LPUART1_PERIPHERAL, &LPUART1_config, LPUART1_CLOCK_SOURCE);
}

/***********************************************************************************************************************
 * Initialization functions
 **********************************************************************************************************************/
void BOARD_InitPeripherals(void)
{
  /* Initialize components */
  FATFS_init();
  LPUART1_init();
}

/***********************************************************************************************************************
 * BOARD_InitBootPeripherals function
 **********************************************************************************************************************/
void BOARD_InitBootPeripherals(void)
{
  BOARD_InitPeripherals();
}
