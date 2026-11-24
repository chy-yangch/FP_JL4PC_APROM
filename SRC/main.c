/******************************************************************************
 * @file     SIK3H.C
 * @brief    NANO1x2 main file
 * @version  0.0.2
 * @date     29, 11, 2016

 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <string.h>
#include <stdint.h>
#include <math.h>
#include "Nano1X2Series.h"
#include "JL4RH.h"
#include "T_Dew_fix.h"
#include "NTC503ET_3H87U30187_100C.C"
#include "I2C_Command.C"
#include "flash_spi.c"
#include "conneco_ble.h"
#include "key_scan.h"

/* 測試用的程式版本改為901,901版本用來測試DATALOG時的省電功能 */
#define FirmwareVersion 1
#define LOG_APO 1							// 	DATALOG 省電功能要設為 1 ; 不省電設為 0
#define DL_data_count 5 /* = 5 for RH; = 8 for PR and PC */

static struct host_event host_event = {
	.status = HOST_IDLE,
	.err.raw = 0
};

static struct button_event button_pending = {
	.status = BUTTON_NONE,
	.id = BUTTON_ID_POWER,
	.duration_ms = 0,
	.ts_hour = 0,
	.ts_min = 0,
	.ts_sec = 0
};

static const int32_t  NTC_2_SHT30_P = 5000;
static const int32_t  NTC_2_SHT30_N = -2000;
static const int32_t  SHT30_2_NTC_P = 4900;
static const int32_t  SHT30_2_NTC_N = -1900;

volatile uint8_t htemp_sw_flag = 0;
volatile uint8_t ltemp_sw_flag = 0;
__attribute__((aligned(4))) static uint8_t data_rh[2];
__attribute__((aligned(4))) static uint8_t data_t[2];

volatile int32_t  ValueTemp = 0;

static uint8_t exit_lpm_meas = 0; /* 離開省電與量測，加速無線傳輸用 */
static uint8_t sn_written = 0; /* 判斷寫完序號 -> 功率後關機用 */

static void ble_enable_uart_pin(uint8_t enable);
static void ble_get_dynamic_data(struct ble_dynamic_data *data);
static void ble_setting_changed(const struct user_setting *set);
static void ble_app_ota_event(const struct ble_ota_params *params);
static void ble_set_log(const struct log_setup *setup, uint8_t *batt_percent);
static void ble_get_log_data(uint32_t addr, uint8_t *data);
static void ble_handle_sn(enum ble_sn_status st, struct product_info *info);

void power_off_status(void);
static void trigger_apo_blink(void);

void SHT40_init(void);
volatile int32_t SHT40_Temp_C = 0,SHT40_Temp_F = 0;
volatile int32_t SHT40_Humi = 0;

struct ble_context ble_ctx = {
	.product = {
		.sn = {0x26, 0x08, 0x12, 0x34},
		.hw_ver = "AA",
		.p_code = "BH",
		.cal_date = {0x20, 0x26, 0x08, 0x14},
		.fw_ver = FirmwareVersion
	},

	.hw = {
		.uart_port = 1,
		.pdma_rx_ch = 2,
		.pdma_tx_ch = 3,

		.rx = {PC, 8},
		.tx = {PC, 7},
		.en = {PC, 9},
		.wakeup = {PC, 6},
		.n_rst = {PC, 3},
		.chy_gatt_active = {PC, 5},
		.ota_active = {PC, 4},
	},

	.cb = {
		.enable_uart_pin = ble_enable_uart_pin,
		.get_dynamic_data = ble_get_dynamic_data,
		.setting_changed = ble_setting_changed,
		.app_ota_event = ble_app_ota_event,
		.set_log = ble_set_log,
		.get_log_data = ble_get_log_data,
		.handle_sn = ble_handle_sn
	},

	.info = {0},
	.state = BLE_STATE_POWER_OFF
};

/* 啟動 / 關閉 UART 功能腳 */
static void ble_enable_uart_pin(uint8_t enable)
{
	SYS_UnlockReg();
	if (enable) {
		SYS->PC_L_MFP |= SYS_PC_L_MFP_PC7_MFP_UART1_RX;
		SYS->PC_H_MFP |= SYS_PC_H_MFP_PC8_MFP_UART1_TX;
	} else {
		SYS->PC_L_MFP &= ~SYS_PC_L_MFP_PC7_MFP_UART1_RX;
		SYS->PC_H_MFP &= ~SYS_PC_H_MFP_PC8_MFP_UART1_TX;
	}
	SYS_LockReg();
}

/*
 * @brief: 將產品的量測值、設定值傳給 BLE 模組
 * @input:
 *    *data: 產品量測值，電池電量、主機/按鍵狀態等...
 * @return: N/A
 */
static void ble_get_dynamic_data(struct ble_dynamic_data *data)
{
	data->batt_level = batt_level;
	data->r_b = indicator;
	data->meas.rh.db = Dry_bulb_Value / 10;
	data->meas.rh.wb = Wb_Value / 10;
	data->meas.rh.humidity = humdity_value;
	data->meas.rh.dp = Dew_T_Value / 10;
	data->meas.rh.enthalpy = (int16_t)Enthalpy;
	data->rp_level = 0;
	data->apo = apo_flag;
	data->time[0] = RTC_data[2]; /* RTC_Hour */
	data->time[1] = RTC_data[1]; /* RTC_min */
	data->time[2] = RTC_data[0]; /* RTC_sec */

	if (host_event.err.raw > 0)
		host_event.status = HOST_ERROR;
	else if (Data_Log_Start_Flag)
		host_event.status = HOST_DATALOG;
	else
		host_event.status = HOST_READY;

	data->host = host_event;

	memset((uint8_t *)&data->button, 0, sizeof(data->button));
	if (button_pending.status > BUTTON_NONE) {
		data->button = button_pending;
		button_pending.status = BUTTON_NONE;
	}
}

/*
 * @brief: 透過 App 更改 & 儲存產品設定值
 * @input:
 *    *set: 產品設定值: APO, R/B
 * @return: N/A
 */
static void ble_setting_changed(const struct user_setting *set)
{
	if (apo_flag != set->apo) {
		apo_flag = set->apo;
		trigger_apo_blink();
	}

	indicator = set->r_b;

	struct saved_info *save = (struct saved_info *)&cali_value_st;
	uint8_t i = 0;
	uint8_t len = sizeof(struct saved_info);

	if (save->apo != apo_flag || save->r_b != indicator) {
		save->apo = apo_flag;
		save->r_b = indicator;
		save->cs = 0;
		for (i = 0; i < len - sizeof(save->cs); i++)
			save->cs += *((uint8_t *)save + i);

		write_fmc1(1, 1, 0x7E00);
	}
}

static void ble_app_ota_event(const struct ble_ota_params *params)
{
	static uint32_t flash_file_size = 0;

	switch (params->event) {
	case BLE_OTA_STARTED:
		/* 清除外部 flash fw image 區 */
		exit_lpm_meas = 1;

		//Erase Flash OTA Space 32K
		bin_space_erase();

		flash_file_size = 0;

		break;
	case BLE_OTA_DATA:
		/* 將 APP 傳送 params->data 寫入外部 flash，長度 params->len */

		W25Q_Write(OTA_BIN_START_ADDR + flash_file_size,
			   params->data, params->len);

		flash_file_size += params->len;

		break;
	case BLE_OTA_FINISHED:
		/* 通知 OTA 已完成 */

		// Enable Write RTC RAM
		RTC->CAR = 0x0000A965;
		//Write RTC RAM為0x02 for flash to update APROM
		RTC->SPR0 = 0x02;

		//切換到Bootloader動作1
		SYS_UnlockReg();
		// 2. 開啟 FMC ISP 功能
		//FMC->ISPCTL |= FMC_ISPCTL_ISPEN_Msk;
		FMC->ISPCON |=  FMC_ISPCON_ISPEN_Msk;
		// 3. 強制指定 Boot Selection 為 LDROM (BS = 1)
		//FMC->ISPCTL |= FMC_ISPCTL_BS_Msk;
		FMC_SET_LDROM_BOOT();
		// 4. 改用 MCU 重設（這會保留 BS=1 的設定並直接跳進 LDROM）
		SYS_ResetCPU();

		while(1);

	}
}

static void ble_set_log(const struct log_setup *setup, uint8_t *batt_percent)
{
	uint8_t i = 0;

	/* 產品 log 變數 = setup */
	Record_mode = setup->mode;
	if (Record_mode)
		ble_power_off();

	/* 設定現在時間 */
	for (i = 0; i < 6; i++)
		day_time[5 - i] = setup->present_time[i];

	Rtc_Set();

	/* 預約 log 開始時間 */
	for (i = 0; i < 6; i++)
		Start_time[5 - i] = setup->start_time[i];

	Rtc_Alarm_Set();

	Log_interval_Sec = setup->interval_sec[0] << 8;
	Log_interval_Sec |= setup->interval_sec[1];

	Duration_Min = setup->duration_min[0] << 8;
	Duration_Min |= setup->duration_min[1];

	/* 取得 local 電池 % */
	*batt_percent = BattPercent;

	/* 清除 Ext Flash */
	Erase_flag = 1;
	exit_lpm_meas = 0;
}

static void ble_get_log_data(uint32_t addr, uint8_t *data)
{
	/*
	 * addr
	 *   = 0: log 設定值, 記錄筆數
	 *   = 1: log 產品設定如 S/N, HW ver 等
	 *   = 2~: log 資料
	 * *data: 傳送給 App 的封包
	 */
	uint8_t i = 0;

	switch (addr) {
	case 0:
		for (i = 0; i < 12; i++)
			*(data + i) = SpiFlash_NormalRead(i);

		/* 取得記錄筆數 */
		Log_Data_Address_Calculate();
		if (DL_Calculate_Exit != 3)
			Log_times_count -= 32;

		for (i = 0; i < 4; i++)
			data[12 + i] = *((uint8_t *)&Log_times_count + 3 - i);

		exit_lpm_meas = 1;
		break;
	case 1:
		for (i = 0; i < 16; i++)
			data[i] = SpiFlash_NormalRead(16 + i);
		break;
	default:
		if (addr > Data_count + 1) {
			for (i = 0; i < 4; i++)
				data[i] = *((uint8_t *)&Log_times_count + 3 - i);

			memcpy(&data[4], "En", 2);
			exit_lpm_meas = 0;
			DataLogEnd_Flag = 1;
		} else {
			uint32_t calc_addr = addr * DL_data_count +
					     (8 - DL_data_count) * 2 + 16;
			uint16_t read = 0;

			for (i = 0; i < DL_data_count; i++) {
				read = SpiFlash_NormalRead(i + calc_addr) - 3480;
				memcpy(&data[i * 2], (uint8_t *)&read, 2);
			}
		}
		break;
	}
}

/*
 * @brief: 處理藍牙連線中讀/寫/清除序號指令
 * @input:
 *    st: 讀/寫/清除序號狀態
 *    *info: 封包傳入的產品資訊
 * @return: N/A
 */
static void ble_handle_sn(enum ble_sn_status st, struct product_info *info)
{
	struct saved_info *save = (struct saved_info *)&cali_value_st;
	uint8_t i = 0;
	uint8_t len = sizeof(struct saved_info);

	memset(save->sn, 0, len);
	save->apo = 1;

	switch (st) {
	case BLE_WRITE_SN:
		/* update product info */
		sn_written = 1;
		memcpy(save->sn, info->sn, 4);
		memcpy(save->cal_date, info->cal_date, 4);
		memcpy(save->hw_ver, info->hw_ver, 2);
		for (i = 0; i < len - sizeof(save->cs); i++)
			save->cs += *((uint8_t *)save + i);
		/* fall through */
	case BEL_CLEAN_SN:
		write_fmc1(1, 1, 0x7E00);
		break;
	default:
		break;
	}

	/* reply product info */
	read_fmc1(1, 1, 0x7E00);

	uint16_t cs = 0;
	for (i = 0; i < len - sizeof(save->cs); i++)
		cs += *((uint8_t *)save + i);

	if (cs == save->cs) {
		memcpy(info->sn, save->sn, 4);
		memcpy(info->cal_date, save->cal_date, 4);
		memcpy(info->hw_ver, save->hw_ver, 2);
		memcpy(info->p_code, ble_ctx.product.p_code, 2);
	}
}

int32_t main(void)
{
	Device_Init();
	while (power_key.key_hold_cnt < 9)
		continue;

	VB_EN = 1;
	ble_init(&ble_ctx);
	ble_power_on();
	Devie_Default();

	switch (SpiFlash_ReadMidDid()) {
	case FLASH_ERR_SPI:
		host_event.err.bit.spi = 1;
		break;
	case FLASH_ERR_DEVICE:
		host_event.err.bit.ext_flash = 1;
		break;
	default:
		break;
	}

	while (1) {
		ble_process();
		if (!exit_lpm_meas && !StopDataLog_Flag) {
			SHT40_init();
			rh1 = read_Humi();
			humidity();
			Battery();
		}
		Dew_T = Dew_Point(T_disp, humdity_value);
		wb = Web_Bulb(T_disp);
		chk_cf();
		Enthalpy = enthalpy_sub(Dry_bulb_Value, Wb_Value);
		Log_Data();
		power_off_status();

		ad_read = 0;
		while (!exit_lpm_meas && !StopDataLog_Flag) {
			CLK->PWRCTL |= (CLK_PWRCTL_PD_EN_Msk);
			__WFI();
			if ((KEY_PWR) || (ad_read))
				break;
		}
		Batt_Life();
		/* WDT_RESET */
		SYS_UnlockReg();
		WDT->CTL |= WDT_CTL_WTR_Msk;
		SYS_LockReg();
	}
}

//-------------------------------------------------------------------------------------------
// Device Initial Setting
//-------------------------------------------------------------------------------------------
void Device_Init(void)
{
	int32_t i32TimeOutCnt;//,i;

	/* Init System Clock  */
	/* Unlock protected registers */
	SYS_UnlockReg();
	WDT->CTL = WDT_TIMEOUT_2POW18 | WDT_CTL_WTE_Msk | WDT_CTL_WTRE_Msk;
	CLK->PWRCTL  |= (CLK_PWRCTL_HIRC_EN  | CLK_PWRCTL_LXT_EN);
	CLK->PWRCTL  &=  ~(CLK_PWRCTL_HXT_EN);
	i32TimeOutCnt = __HSI / 200;
	while ((CLK->CLKSTATUS & CLK_CLKSTATUS_HIRC_STB_Msk) !=
	       CLK_CLKSTATUS_HIRC_STB_Msk) {
		if (i32TimeOutCnt-- <= 0)
			__NOP();
	}
	i32TimeOutCnt = __HSI / 200;
	while ((CLK->CLKSTATUS & CLK_CLKSTATUS_LXT_STB_Msk) !=
	       CLK_CLKSTATUS_LXT_STB_Msk) {
		if (i32TimeOutCnt-- <= 0)
			__NOP();
	}
	CLK->CLKSEL0 |= CLK_CLKSEL0_HCLK_S_HIRC;
	CLK->APBCLK = CLK_APBCLK_TMR0_EN + CLK_APBCLK_TMR1_EN +
		      CLK_APBCLK_TMR2_EN + CLK_APBCLK_ADC_EN +
		      CLK_APBCLK_TMR3_EN + CLK_APBCLK_RTC_EN +
		      CLK_APBCLK_WDT_EN + CLK_APBCLK_UART0_EN +
		      CLK_APBCLK_UART1_EN + CLK_APBCLK_SPI1_EN;
	CLK->AHBCLK  |= CLK_AHBCLK_DMA_EN_Msk;
	CLK->CLKDIV0 |= CLK_HCLK_CLK_DIVIDER(6) + CLK_ADC_CLK_DIVIDER(2);
	CLK->CLKDIV1 |= CLK_TMR0_CLK_DIVIDER(1);
	CLK->CLKSEL1 &= ~(BIT14 + BIT10);
	CLK->CLKSEL1 |= BIT12 + BIT8;
	CLK->CLKSEL2 &= ~(BIT14 + BIT10);
	CLK->CLKSEL2 |= BIT12 + BIT8;

	SYS->Int_VREFCTL |= (SYS_VREFCTL_EXT_MODE_Msk | SYS_VREFCTL_SEL25_Msk |
			     SYS_VREFCTL_REG_EN_Msk | SYS_VREFCTL_BGP_EN_Msk);
	SYS->LDO_CTL = 0x00000B04;


	/* Init I/O Multi-function  */
	/*
	 * PA15 SPI1_SS0
	 * PA14 SPI1_CLK
	 * PA13 SPI1_MISO0
	 * PA12 SPI1_MOSI0
	 * PA11 ---
	 * PA10 ---
	 * PA9 ---
	 * PA8 ---
	 * PA7 ---
	 * PA6 FCS
	 * PA5 /TSL
	 * PA4 NC
	 * PA3 POWER KEY
	 * PA2 BATT IN
	 * PA1 NC
	 * PA0 NC
	 */
	PA->DOUT = 0x00000000;
	SYS->PA_H_MFP = 0x66660000;
	SYS->PA_L_MFP = 0x00000200;
	PA->PMD = 0x55555505;
	PA->OFFD = 0x00070000;
	PA->PUEN = 0x00000048;
	PA->DBEN = 0x00000000;

	/*
	 * PB15 ----
	 * PB14 TXD			O
	 * PB13 RXD			I
	 * PB12 ON_BYP			O
	 * PB11 ----
	 * PB10 ----
	 * PB9 	----
	 * PB8 	----
	 * PB7 	----
	 * PB6 	----
	 * PB5 	----
	 * PB4 	----
	 * PB3 	----
	 * PB2 	----
	 * PB1 	----
	 * PB0 	----
	 */
	PB->DOUT = 0x00000000;
	SYS->PB_H_MFP = 0x07700000;
	PB->PMD =  0x51000000;
	PB->OFFD = 0x00000000;
	PB->PUEN = 0x00000000;


	/*
	 * PC15 ----
	 * PC14 APO					O
	 * PC13 ----
	 * PC12 ----
	 * PC11 ----
	 * PC10 ----
	 * PC9 BLE_EN O
	 * PC8 UART1_TXD BLE_RX USE O
	 * PC7 UART1_RXD BLE_TX USE I
	 * PC6 BLE_WAKE_UP O
	 * PC5 BLE_CHY_GATT_ACTIVE I
	 * PC4 BLE_OTA_ACTIVE I
	 * PC3 BLE_NRST O
	 * PC2 ADJ I
	 * PC1 IO_SDA0 	HTU21D  use	IO
	 * PC0 IO_SCL0 	HTU21D  use	IO
	 */
	SYS->PC_H_MFP = 0x00000000;
	SYS->PC_L_MFP = 0x00000000;
	PC->PMD  = 0x55551015;
	PC->OFFD = 0x00000000;
	PC->PUEN = 0x00000000;

	/*
	 * PD15 WR 	HT1621 LCD USE 	IO
	 * PD14 CS 	HT1621 LCD USE	IO
	 * PD13 No USE	IO
	 * PD12 ----
	 * PD11 ----
	 * PD10 RED		O PWM
	 * PD9 	GREEN	O	PWM
	 * PD8 	DATA	HT1621 LCD USE	IO
	 * PD7 	ADJ
	 * PD6 	----
	 * PD5 	----
	 * PD4 	----
	 * PD3 	----
	 * PD2 	----
	 * PD1 	----
	 * PD0 	----
	 */
	PD->DOUT = 0x00000000;
	SYS->PD_H_MFP = 0x00000000;
	SYS->PD_L_MFP = 0x00000000;
	PD->PMD =  0x55555555;
	PD->OFFD = 0x00000000;
	PD->PUEN = 0x00000000;

	/*
	 * PF5	ICE_DAT		(Prog)
	 * PF4	ICE_CLK		(Prog)
	 * PF3	NC				O
	 * PF2	NC				O
	 * PF1	NC				O
	 * PF0	NC				O
	 */
	PF->DOUT = 0x00000000;
	SYS->PF_L_MFP = 0x00FF0020;
	PF->PMD   = 0XFFFFF055;
	PF->OFFD = 0x00000000;
	PF->PUEN = 0x00000000;


	SYS_LockReg(); //To lock the protected register to forbid write access
	SPI_Open(SPI_FLASH_PORT, SPI_MASTER, SPI_MODE_0, 8, 2000000);
	/* Enable the automatic hardware slave select function. Select the SS0 pin and configure as low-active. */
	SPI_EnableAutoSS(SPI_FLASH_PORT, SPI_SS0, SPI_SS0_ACTIVE_LOW);
	SPI_EnableFIFO(SPI_FLASH_PORT, 4, 4);
	/* Wait ready */
	SpiFlash_WaitReady();

	APO_ON = 0; 		//Enable VCC power

	/* 10/sec */
	TIMER0->PRECNT = 0x00000000;
	TIMER0->CMPR = 3277 - 1;
	TIMER0->CTL = TIMER_CTL_TMR_EN_Msk | TIMER_PERIODIC_MODE;
	TIMER0->IER |= TIMER_IER_TMR_IE_Msk;
	NVIC_EnableIRQ(TMR0_IRQn);
	/* 20/sec */
	TIMER1->PRECNT = 0x00000000;
	TIMER1->CMPR = 1638 - 1;
	TIMER1->CTL = TIMER_CTL_TMR_EN_Msk | TIMER_PERIODIC_MODE;
	TIMER1->IER |= TIMER_IER_TMR_IE_Msk;
	NVIC_EnableIRQ(TMR1_IRQn);
	/* 100/sec */
	TIMER2->PRECNT = 0x00000000;
	TIMER2->CMPR = 32768 - 1;
	TIMER2->CTL = TIMER_CTL_TMR_EN_Msk | TIMER_PERIODIC_MODE;
	TIMER2->IER |= TIMER_IER_TMR_IE_Msk;
	NVIC_EnableIRQ(TMR2_IRQn);
	/* 4/sec */
	TIMER3->PRECNT = 0x00000000;
	TIMER3->CMPR = 8192 - 1;
	TIMER3->DR = 0;
	TIMER3->CTL = TIMER_CTL_TMR_EN_Msk | TIMER_PERIODIC_MODE;
	TIMER3->IER |= TIMER_IER_TMR_IE_Msk;
	NVIC_EnableIRQ(TMR3_IRQn);

	ADC->CR = (ADC->CR & ~ADC_CR_DIFF_Msk) | ADC_INPUT_MODE_SINGLE_END;
	ADC->CR |= (ADC_CR_ADMD_Msk + ADC_RESSEL_12_BIT + ADC_REFSEL_INT_VREF);
	ADC->SMPLCNT0 = 0xbbbbbbbb;
	ADC->CHEN |= ADC_CH_2_MASK;
	ADC_POWER_ON(adc);
	ADC->CR |= ADC_CR_PTEN_Msk;

	PDMA_Init();
	PA->IMD = 0x00000000;
//	PA->IER |= 0x00080000;
//	PA->DBEN = 0x00000008;
//	NVIC_EnableIRQ(EINT1_IRQn);


	/* Baud rate: 19200, 8, N, 1 */
	UART0->BAUD = (UART_BAUD_MODE0 | UART_BAUD_MODE0_DIVIDER(12000000,
			19200));
	UART0->TLCTL = UART_WORD_LEN_8 | UART_PARITY_NONE |
		       UART_STOP_BIT_1 | UART_TLCTL_RFITL_1BYTE
		       | UART_TLCTL_RTS_TRI_LEV_1BYTE;
	NVIC_EnableIRQ(UART0_IRQn);
	UART0->IER   = UART_IER_RTO_IE_Msk | UART_IER_RDA_IE_Msk;

	Set_default_value();
	RTC->AER = RTC_WRITE_KEY;
	while (!(RTC->AER & RTC_AER_ENF_Msk));
	RTC->TTR = 0x0000;
	RTC_EnableInt(RTC_RIER_TIER_Msk);
	NVIC_EnableIRQ(RTC_IRQn);

	CLK_SysTickDelay(500);			//max=2^24/MLK 1M =>16777216/1000000=16777216 us

	SystemCoreClockUpdate();
}

void ClearFlag(void)
{
	Power_On_Flag = 0;
	Wake_Log_Flag = 0;

	APOcounter = 0;
	apo_select = 0;

	batt_led_flag = 0;
	key_off = 0;
	cali_flag = 0;

	BattPercent = 0;
	BattLifeM = 0;
	BattLifeLow_Flag = 0;

	apo_select = 0;


	key_5s_cont = 0;

	ad_cont = 0;
	ad_read = 0;
	RLED = 0;
	BLED = 0;



	if (!Record_mode) {
		TIMER2->CTL    &= ~TIMER_CTL_WAKE_EN_Msk ;
		TIMER3->CTL    &= ~TIMER_CTL_WAKE_EN_Msk ;
		GLED = 0;
		led_mode = 0;
	}
	if (StopDataLog_Flag) {
		Data_Log_Start_Flag = 0;
		Data_Log_reservation_Flag = 0;
		duration_cont = 0;
		Record_mode = 0;
		StopDataLog_Flag = 0;
	}
}


void Set_default_value(void)
{
	if (Set_default_Flag) {
		day_time[5] = 0x17;
		day_time[4] = 0x03;
		day_time[3] = 0x02;
		day_time[2] = 0x08;
		day_time[1] = 0x30;
		day_time[0] = 0x00;
		Rtc_Set();
		Set_default_Flag = 0;
	}
}

void Data_Log_File0(void)
{
	volatile uint8_t dl_sum_cont = 0;
	volatile uint32_t dl_sum = 0;

	File_Inf0[0] = 0x00;
	File_Inf0[1] = 0x00;
	File_Inf0[2] = Start_time[5];		//Start_time Year  M0
	File_Inf0[3] = Start_time[4];		//Start_time month
	File_Inf0[4] = Start_time[3];		//Start_time day
	File_Inf0[5] = Start_time[2];		//Start_time hour
	File_Inf0[6] = Start_time[1];		//Start_time minitue
	File_Inf0[7] = Start_time[0];		//Start_time second
	File_Inf0[8] = Record_mode;  // Reserved
	File_Inf0[9] = (Log_interval_Sec & 0x0000FF00) >> 8;
	File_Inf0[10] = Log_interval_Sec & 0x000000FF;	/* 間隔時間 */
	File_Inf0[11] = (Duration_Min & 0x0000FF00) >> 8;
	File_Inf0[12] = Duration_Min & 0x000000FF;	/* 持續時間 */
	File_Inf0[13] = Log_SET;  // Reserved

	for (dl_sum_cont = 2; dl_sum_cont < 18; dl_sum_cont++) {
		dl_sum += File_Inf0[dl_sum_cont];
		File_Inf0[19] = dl_sum;
		if (dl_sum_cont == 17) {
			File_Inf0[18] = dl_sum >> 8;
			dl_sum = 0;
		}
	}
}

void Data_Log_File1(void)
{
	volatile uint8_t dl1_sum_cont = 0;
	volatile uint32_t dl1_sum = 0;

	File_Inf1[0] = 0x00;
	File_Inf1[1] = 0x01;
	File_Inf1[2] = Serial_Number[0];
	File_Inf1[3] = Serial_Number[1];
	File_Inf1[4] = Serial_Number[2];
	File_Inf1[5] = Serial_Number[3];
	File_Inf1[6] = ble_ctx.product.hw_ver[0];
	File_Inf1[7] = ble_ctx.product.hw_ver[1];
	File_Inf1[8] = ble_ctx.product.p_code[0];
	File_Inf1[9] = ble_ctx.product.p_code[1];
	File_Inf1[10] = indicator;
	File_Inf1[11] = 0;
	File_Inf1[12] = 0;
	File_Inf1[13] = 0;
	File_Inf1[14] = 0;
	File_Inf1[15] = 0;
	File_Inf1[16] = 0;
	File_Inf1[17] = 0;

	for (dl1_sum_cont = 2; dl1_sum_cont < 18; dl1_sum_cont++) {
		dl1_sum += File_Inf1[dl1_sum_cont];
		File_Inf1[19] = dl1_sum;
		if (dl1_sum_cont == 17) {
			File_Inf1[18] = dl1_sum >> 8;
			dl1_sum = 0;
		}
	}
}

uint16_t Address_Calculate(uint32_t addc)
{
	uint16_t out;

	DL_times = addc; //1048576   先判斷中間位址是有有資料.
	out = SpiFlash_NormalRead(DL_times);
	SpiFlash_WaitReady();
	return out;
}

void Log_Data_Address_Calculate(void)
{
	volatile uint8_t LD0, DL_C_S;

	DL_Calculate_Exit = 0;
	LD0 = 0;
	/* 大範圍判斷寫入FLASH的位址 */
	while (!DL_Calculate_Exit && !Power_Off) {
		SYS_UnlockReg();
		WDT->CTL  |= WDT_CTL_WTR_Msk;
		SYS_LockReg();
		/* (2^LD0); 除2的N次方 */
		Log_times_count = DL_Address / (pow(2, LD0));
		DL_Calculate = Address_Calculate(Log_times_count);//載入SET0位址 ;524288
		if (DL_Calculate == 0XFFFF)
			LD0++;
		else
			DL_Calculate_Exit = 1;

		if (Log_times_count == 0 && LD0 > 20)
			DL_Calculate_Exit = 3;
	}
	/* 小範圍判斷寫入FLASH的位址 */
	while (DL_Calculate_Exit == 1 && !Power_Off) {
		SYS_UnlockReg();
		WDT->CTL  |= WDT_CTL_WTR_Msk;
		SYS_LockReg();
		DL_Calculate = Address_Calculate(Log_times_count);
		if (DL_Calculate == 0XFFFF) {
			DL_Calculate_Exit = 2;
			Data_count = (Log_times_count - 32) % DL_data_count; //8->5
			if (Data_count != 0)
				Data_count = ((Log_times_count - 32)  / DL_data_count) + 1; //8->5
			else
				Data_count = ((Log_times_count - 32)  / DL_data_count) + 0; //8->5
		} else {
			Log_times_count++;
			DL_Calculate_Exit = 1;
		}
	}
}


void Log_Data(void)
{
	volatile int32_t i = 0, j = 0;
	volatile int32_t FI0 = 0, FI1 = 0;


	if (Flash_write_flag) {
		if (File_Inf0_Flag) {
			File_Inf0_Flag = 0;
			Data_Log_File0();
			for (FI0 = 2; FI0 < 14; FI0++)	//暫存器14~17 , FLASH 位址 12,13,14,15先不記錄
				SpiFlash_NormalPageProgram(FI0 - 2,  File_Inf0[FI0]);
		}
		if (File_Inf1_Flag) {
			File_Inf1_Flag = 0;
			Data_Log_File1();
			for (FI1 = 2; FI1 < 18; FI1++)
				SpiFlash_NormalPageProgram(14 + FI1,  File_Inf1[FI1]);

			log_addr = 32; //寫入檔頭資訊後,從第32個位址開始寫入 量測資料.
		}

		Flash_value = (int16_t)(Dry_bulb_Value / 10) + 3480; //固定加上一個定值,因為若是負值,會有判斷上的問題
		/* 固定加上一個定值,因為若是負值,會有判斷上的問題 */
		// 每次寫入,由每個SET的第31個位址開始寫入 (DL_times=2),0~31分別寫入檔頭資訊0和1
		SpiFlash_NormalPageProgram(log_addr,  Flash_value);  //寫入FLASH 一次寫入2Byte ,
		log_addr++;

		Flash_value = (humdity_value / 1) + 3480; //固定加上一個定值,因為若是負值,會有判斷上的問題
		/* 寫入FLASH 一次寫入2Byte */
		SpiFlash_NormalPageProgram(log_addr,  Flash_value);
		log_addr++;
		Flash_value = (int16_t)(Wb_Value / 10) + 3480;
		SpiFlash_NormalPageProgram(log_addr,  Flash_value);
		log_addr++;
		Flash_value = (int16_t)(Dew_T_Value / 10) + 3480;
		SpiFlash_NormalPageProgram(log_addr,  Flash_value);
		log_addr++;
		Flash_value = (int16_t)Enthalpy + 3480;
		SpiFlash_NormalPageProgram(log_addr,  Flash_value);
		log_addr++;
		Flash_write_flag = 0;
	}
	if (Flash_read_flag) {
		SpiFlash_WaitReady();
		for (i = 0; i < 16; i++) //onetime log超過一圈的DL.
			DL_logged_data_array[i] = SpiFlash_NormalRead(i + (DL_times *
						  16)); //2byte	(0x000000); //讀出FLASH

		Flash_read_flag = 0;
	}
	if (Erase_flag)	{
		SpiFlash_ChipErase_logger(0);    //Erase 0 sets    0x0000000  ~  0x0FFFFF
		SpiFlash_ChipErase_logger(1);    //Erase 1 sets    0x0100000  ~  0x1FFFFF
		SpiFlash_ChipErase_logger(2);    //Erase 0 sets    0x0200000  ~  0x2FFFFF
		SpiFlash_ChipErase_logger(3);    //Erase 1 sets    0x0300000  ~  0x3FFFFF
		SpiFlash_WaitReady();
		File_Inf0_Flag = 1;
		File_Inf1_Flag = 1;
		Data_Log_Start_Flag = 1;
		Data_Log_reservation_Flag = 0;
		Erase_flag = 0;
		host_event.status = HOST_DATALOG;
	}

	if (Download_Flag) // 記算LOG筆數
		Log_Data_Address_Calculate();

	if (Data_Log_Start_Flag && led_mode != 10) {
		if (!Data_Log_reservation_Flag)
			led_mode = 7;
		else
			led_mode = 8;
	}

	if (DataLogEnd_Flag == 1) {
		DataLogEnd_Flag = 0;
		Power_Off = 1;
	}
}




void Devie_Default(void)
{
	StopDataLog_Flag = 0;
	Power_Off = 0;
	pkey_mode = 1;
	key_5s_cont = 0;
	htemp_sw_flag = 0;
	ltemp_sw_flag = 0;

	if (!Wake_Log_Flag)
		GLED = 1;

	cali_sub();
	SHT40_init();
	CLK_SysTickDelay(100);

	key_off = 0;
	cf_flag = 0;
	APO_ON = 0;
	VB_EN = 1;

	if (!Data_Log_Start_Flag && !Data_Log_reservation_Flag) {
		if (led_mode != 10) {
			led_mode = 1;
			led_count = 0;
			TIMER2->CMPR   = 32768 - 1;
			TIMER2->DR =  0;
			NVIC_EnableIRQ(TMR2_IRQn);
			led_count1 = 0;
			TIMER3->CMPR   = 8192 - 1;									////每秒 4次
			TIMER3->DR =  0;
			TIMER3->CTL    |= TIMER_CTL_TMR_EN_Msk;//NVIC_EnableIRQ(TMR3_IRQn);
			GLED = 1;
		}
	}
}

void SHT40_init(void)
{
	I2c_StartCondition();
	I2c_WriteByte(0x88);
	I2c_WriteByte(0xFD);
	I2c_StopCondition();
	CLK_SysTickDelay(10000);
}


void SHT40_reset(void)
{
	I2c_StartCondition();
	I2c_WriteByte(0x88);
	I2c_WriteByte(0x94);
	I2c_StopCondition();
	CLK_SysTickDelay(1000);
}

uint8_t SHT40_CRC_Check(uint8_t *data)
{
	uint8_t  bit;
	uint8_t  byteCtr;
	uint16_t crc = 0xFF;
	
	for (byteCtr = 0; byteCtr < 2; byteCtr++) 
	{
		crc ^= (data[byteCtr]);
		for (bit = 8; bit > 0; --bit)	
		{
			if (crc & 0x80)
				crc = (crc << 1) ^ 0x131;
			else
				crc = (crc << 1);
		}
	}
	
	return (uint8_t)crc;
}

int32_t read_Humi(void)
{
	uint16_t rawValue, rawValue_T; // temperature raw value from sensor

	uint8_t  checksum_T, checksum_RH;

	err = 0;

	I2c_StartCondition();
	I2c_WriteByte(0x88);
	I2c_WriteByte(0xFD);
	CLK_SysTickDelay(10000); // 10ms SENSOR可動作
	
	I2c_StartCondition();
	I2c_WriteByte(0x89);
	CLK_SysTickDelay(1);

	data_t[0] = I2c_ReadByte();
	data_t[1] = I2c_ReadByte();
	checksum_T = I2c_ReadByte();

	data_rh[0] = I2c_ReadByte();
	data_rh[1] = I2c_ReadByte();
	checksum_RH = I2c_ReadByte();

	I2c_StopCondition();
	
	if ((SHT40_CRC_Check(data_t) == checksum_T) && (SHT40_CRC_Check(data_rh) == checksum_RH) && (err == 0)) 
	{
		rawValue = (data_rh[0] << 8) | data_rh[1];
		SHT40_Humi = (((1250 * rawValue) / 65535)) - 60;
		rawValue_T = (data_t[0] << 8) | data_t[1];				/*combine the two bytes to a 16-bit value */
		SHT40_Temp_C = ((float)(175 * rawValue_T) / 65535 - 45) * 100;	/* 將16bit的值代入公式,計算出溫度值 (度C公式) ; ((float)(347 * rawValue_T) / 65535 - 49)*100(度F公式);*/
		SHT40_Temp_F = ((float)(315 * rawValue_T) / 65535 - 49) * 100;

	} 
	else 
	{
		SHT40_reset();
	}

	host_event.err.bit.i2c = err;

	T_disp = SHT40_Temp_C;
	
	return  SHT40_Humi;
}


void humidity(void)
{
	int16_t a;
	int32_t hm_dis;
	float rh_dis, rh_dis1;


	/* valueRH[0]=3.489E+3;
	*valueRH[1]=3400.0;
	*valueRH[2]=1.00919652;
	*valueRH[3]=9.74491238E-1;
	*/
	if (cali_flag == 0) {
		
		if (rh1 >= valueRH[0])      {

			rh1 = (int32_t)((float)rh1 - valueRH[0]);
			hm_dis = (int32_t)((float)rh1 * valueRH[2] + valueRH[1]);

		} else {

			hm_dis = (int32_t)((float)rh1 * valueRH[3]);
		}

		humdity_value = (int16_t)(hm_dis + 0.5f) / 1;
		if (humdity_value >= 1000)
			humdity_value = 1000;
		if (humdity_value <= 1)
			humdity_value = 0;
	}

}

void cali_sub(void)
{
	uint8_t i = 0;
	struct saved_info *save = (struct saved_info *)&cali_value_st;

	read_fmc1(1, 1, 0x7E00);
	CLK_SysTickDelay(1000);

	uint16_t cs = 0;
	for (i = 0; i < sizeof(struct saved_info) - sizeof(save->cs); i++)
		cs += *((uint8_t *)save + i);

	if (cs != save->cs) {
		// The S/N has never been writen
		host_event.err.bit.int_flash = 1;
		read_fmc1(1, 2, 0x79FC); // Read template S/N
		Serial_Number[0] = (int8_t)(cali_value_st[0] & 0x000000FF);
		Serial_Number[1] = (int8_t)((cali_value_st[0] & 0x0000FF00) >> 8);
		Serial_Number[2] = (int8_t)((cali_value_st[0] & 0x00FF0000) >> 16);
		Serial_Number[3] = (int8_t)((cali_value_st[0] & 0xFF000000) >> 24);
	} else {
		for (i = 0; i < 4; i++) {
			Serial_Number[i] = save->sn[i];
			Cal_Date[i] = save->cal_date[i];
		}

		for (i = 0; i < 2; i++)
			ble_ctx.product.hw_ver[i] = save->hw_ver[i];

		if (save->apo > 1)
			apo_flag = 1;
		else
			apo_flag = save->apo;

		if (save->r_b > 1)
			indicator = 0;
		else
			indicator = save->r_b;
	}

	for (i = 0; i < 4; i++) {
		ble_ctx.product.sn[i] = Serial_Number[i];
		ble_ctx.product.cal_date[i] = Cal_Date[i];
	}

	valueRH[0] = eep_rword(0); //33%量測值
	valueRH[1] = eep_rword(1); //33%標準值
	valueRH[2] = eep_rword(2); //slope
	valueRH[3] = eep_rword(3); //RH_33

	valueRH[4] = eep_rword(4); //ADC OFFSET
	valueRH[5] = eep_rword(5); //NTC OFFSET

	valueRH[6] = eep_rword(6); //NTC TYPE

	if (valueRH[6] < 1 || valueRH[6] > 23)
		ntc_table = 9;
	else
		ntc_table = (uint8_t)valueRH[6];

	if (valueRH[0] < 0 && valueRH[1] < 0) { /* test */
		valueRH[0] = 3404;//eep_rword(0); //33%量測值
		valueRH[1] = 3160;//eep_rword(1); //33%標準值
		valueRH[2] = 1.05483162;//eep_rword(2); //slope
		valueRH[3] = 0.928319633;//eep_rword(3); //RH_33
		valueRH[4] = 71;//eep_rword(4); //ADC OFFSET
		valueRH[5] = 54;//eep_rword(5); //NTC OFFSET
		valueRH[6] = 9;//eep_rword(5); //NTC TYPE
		host_event.err.bit.cal = 1;
	}
}

//-------------------------------------------------------------------------------------------
// Power function
//-------------------------------------------------------------------------------------------
void power_sub(void)
{
	if (Power_Off) {
		Apo_Exit_Flag = 0;
		apo_io();
	}
}

/******************************************************************************/
/*  ?????6V???????5V                                        */
/*  ???????????, A3??????0.672V???Lowbattery,(1102 Counts)*/
/*                      ??0.6V?,?????,?????????(983 Counts)*/
/*  (4.2 / 11) / (2.5 / 4095) = 1102, (6.6 / 11) / (2.5 / 4095) = 983         */
/******************************************************************************/
void Battery(void)
{
	//uint8_t one;
	uint32_t batt_vol;             // 電池電壓
	//if (apo_select==0)

	batt_vol = adc_read(1, 32, ADC_CH_2_MASK);

	Batt_Voltage = (((float)batt_vol * Batt_Magnification) * 10 + 0.5) /
		       1000.0; //battery percentage use

	if (batt_vol > BATT_100) {
		batt_led_flag = 0;
		batt_level = 4;
	} else if (batt_vol < BATT_100 && batt_vol > BATT_80) {
		batt_led_flag = 0;
		batt_level = 3;
	} else if (batt_vol < BATT_80 && batt_vol > BATT_60) {
		batt_led_flag = 0;
		batt_level = 2;
	} else if (batt_vol < BATT_60 && batt_vol > BATT_40) {
		batt_led_flag = 0;
		batt_level = 1;
		if (led_mode == 4) {
			led_mode = 1;
			RLED = 0;
		}
	} else if (batt_vol < BATT_40 && batt_vol > BATT_20)
		batt_level = 0;
	else if (batt_vol <  BATT_20) {
		batt_led_flag = 1;
		batt_level = 0;
		if (led_mode == 1 && apo_select == 0) {
			led_mode = 4;
			GLED = 0;
		} else if (led_mode == 5 && apo_select == 0)	{
			led_mode = 4;
			GLED = 0;
		}
		key_5s_cont = 0;
	}
}



float Batt_Percentage(float x)  //battery percentage use
{
	//float battV=0;
	//float RBattPercent=0;
//	>= 2.9v               y = -277.03x2 + 1711.1x - 2540.1
//	>=2.8      <2.9v      y = -104.58x2 + 751.87x - 1209.1
//	>=2.6      <2.8v      y = -476.26x2 + 2767.2x - 3938.8
//	>=2.4      <2.6v      y = 315.76x2 - 1463.6x + 1707.8
//	>=2.1      <2.4v      y = 29.433x2 - 87.029x + 53.198

	battV = x;
	if (battV > 2.900)
		RBattPercent = -277.03 * battV * battV + 1711.1 * battV - 2540.1;
	else if (battV > 2.800)
		RBattPercent = -104.58 * battV * battV + 751.87 * battV - 1209.1;
	else if (battV > 2.600)
		RBattPercent = -476.26 * battV * battV + 2767.2 * battV - 3938.8;
	else if (battV > 2.400)
		RBattPercent = 315.76 * battV * battV - 1463.6 * battV + 1707.8;
	else if (battV > 2.100)
		RBattPercent = 29.433 * battV * battV - 87.029 * battV + 53.198;
	else
		RBattPercent = 0;

	RBattPercent = RBattPercent + 0.5;
	if (RBattPercent > 100)
		RBattPercent = 100;
	else if (RBattPercent < 0)
		RBattPercent = 0;

	return RBattPercent;

}

void Batt_Life(void)//battery percentage use
{
	BattPercent = (Batt_Percentage(Batt_Voltage)); //%計算

	BattLifeM = (BattPercent * Batt_Life_Min) / 100;
	if (BattLifeM > Duration_Min)
		BattLifeLow_Flag = 0;
	else
		BattLifeLow_Flag = 1;

}


void temp(void)
{

	volatile int32_t res = 0;
	volatile int8_t i;
	volatile int16_t txd;
	volatile int32_t d, e, f, h, g;

	if (cali_flag == 1)	{
		res = adc_read(40, 128, ADC_CH_2_MASK);
		ntc_offset = res;
	}

	if (start == 3)
		res = adc_read(5, 16, ADC_CH_2_MASK);
	else
		res = adc_read(40, 128, ADC_CH_2_MASK);

	res = res + ntc_offset;
	temp_avg_data[0] = ntc_table_fix(res, (uint8_t)ntc_table);
	temp_avg();
	NTC_disp = temp_avg_data[3];
	if (NTC_disp > NTC_2_SHT30_P) { // 50.8
		if (!htemp_sw_flag) {
			htemp_sw_flag = 1;
			ltemp_sw_flag = 0;
			T_disp = ValueTemp;//NTC_disp;
		}
	} else if (NTC_disp < NTC_2_SHT30_N) { //-20.8
		if (!ltemp_sw_flag) {
			htemp_sw_flag = 0;
			ltemp_sw_flag = 1;
			T_disp = ValueTemp;//NTC_disp;
		}
	} else {
		T_disp = NTC_disp;
	}
}

void temp_avg(void)
{
	int32_t a;

	a = temp_avg_data[3] - temp_avg_data[0];
	if (a < 0)
		a = a * -1;
	if (a < 10)	{
		temp_avg_cont++;
		if (temp_avg_cont >= 5)	{
			temp_avg_cont = 0;
			temp_avg_data[2] = temp_avg_data[1] / 5;
			temp_avg_data[1] = temp_avg_data[0];
			temp_avg_data[3] = (temp_avg_data[3] + temp_avg_data[0] + temp_avg_data[2]) / 3;
		} else {
			temp_avg_data[1] = temp_avg_data[1] + temp_avg_data[0];
			temp_avg_data[3] = (temp_avg_data[3] + temp_avg_data[0] + temp_avg_data[2]) / 3;
		}
	} else {
		temp_avg_data[2] = temp_avg_data[0];
		temp_avg_data[3] = temp_avg_data[0];
		temp_avg_data[1] = temp_avg_data[0];
		temp_avg_cont = 0;
	}
}

int32_t Dew_Point(int32_t x, int16_t y)
{
	float humi;
	int32_t est;
	int16_t b;
	int32_t DewT;//,NTC;

	humi = (float)y / 1000.0; //%RH
	tdisp = x;

	if (tdisp >= 7000)
		b = 6999;
	else {
		if (tdisp <= -2200)
			b = -2199;
		else
			b = tdisp;
	}
	est = T_Dew_fix(b);
	est1 = (float)est * humi;
	DewT = (int32_t)Dew_T_fix((int32_t)est1);
	if (tdisp < 0)
		DewT = DewT + tdisp / 10;

	return DewT;
}

int32_t Web_Bulb(int32_t x) //void Dew2T(void)
{
	float ewf_1, ewf_2, ewt_1, ewt_2, a;
	int32_t tdisp_1 = 0, wb0;
	int8_t y = 0;

	tdisp = x;
	if (tdisp >= 7000)
		y = 1;
	else {
		if (tdisp <= -2200)
			y = 1;
		else
			y = 0;
	}

	if (y == 0) {
		while (1) {
			tdisp_1 = tdisp_1 + 500;
			ewf_1 = est1 + 0.00662 * 101325 * (tdisp - (tdisp - tdisp_1)) *
				(1 + 0.0000115 * (tdisp - tdisp_1));
			ewt_1 = (float)T_Dew_fix(tdisp - tdisp_1);
			ewf_2 = est1 + 0.00662 * 101325 * (tdisp - (tdisp - tdisp_1 - 500)) *
				(1 + 0.0000115 * (tdisp - tdisp_1 - 100));
			ewt_2 = (float)T_Dew_fix(tdisp - tdisp_1 - 500);
			if (ewf_2 > ewt_2)
				break;
		}
		tdisp_1 = tdisp_1 - 500;
		while (1) {
			tdisp_1 = tdisp_1 + 100;
			ewf_1 = est1 + 0.00662 * 101325 * (tdisp - (tdisp - tdisp_1)) *
				(1 + 0.0000115 * (tdisp - tdisp_1));
			ewt_1 = (float)T_Dew_fix(tdisp - tdisp_1);
			ewf_2 = est1 + 0.00662 * 101325 * (tdisp - (tdisp - tdisp_1 - 100)) *
				(1 + 0.0000115 * (tdisp - tdisp_1 - 100));
			ewt_2 = (float)T_Dew_fix(tdisp - tdisp_1 - 100);
			if (ewf_2 > ewt_2) {
				a = (tdisp - tdisp_1) - ((ewt_1 - ewf_1) / ((ewf_2 - ewf_1) +
							 (ewt_1 - ewt_2))) * 100;
				wb0 = (int32_t)a;
				break;
			}
		}
		if (tdisp <= 500)
			wb0 = wb0 - 10;
	}
	return wb0;
}

void chk_cf(void)
{
	//cf_flag=1;
	if (cf_flag == 1) {
		if (tdisp < 0)
			Dry_bulb_Value = (T_disp - 5) / 1;
		else
			Dry_bulb_Value = (T_disp + 5) / 1;
		if (Dew_T < 0)
			Dew_T_Value = (Dew_T - 5) / 1;
		else
			Dew_T_Value = (Dew_T + 5) / 1;
		if (wb < 0)
			Wb_Value = (wb - 5) / 1;
		else
			Wb_Value = (wb + 5) / 1;
	} else {
		Dry_bulb_Value = (int16_t)((float)T_disp * 1.8 + 3200);
		Dew_T_Value = (int16_t)((float)Dew_T * 1.8 + 3200);
		Wb_Value = (int16_t)((float)wb * 1.8 + 3200);
		/*Wb_Value = (int16_t)((float)ValueTemp * 1.8 + 3200);*/
		if (Dry_bulb_Value < 0)
			Dry_bulb_Value = (Dry_bulb_Value - 5) / 1;
		else
			Dry_bulb_Value = (Dry_bulb_Value + 5) / 1;
		if (Dew_T_Value < 0)
			Dew_T_Value = (Dew_T_Value - 5) / 1;
		else
			Dew_T_Value = (Dew_T_Value + 5) / 1;
		if (Wb_Value < 0)
			Wb_Value = (Wb_Value - 5) / 1;
		else
			Wb_Value = (Wb_Value + 5) / 1;
	}
}

int32_t adc_read(uint16_t x, uint16_t y, uint32_t start_ch)
{
	uint8_t i = 0, j;
	int32_t res = 0;
	int32_t a;

	ADC_START_CONV(adc);
	CLK_SysTickDelay(500);
	PDMA_Trigger(PDMA_CH);

	if (cali_flag >= 1) {
		a = 0;
	} else {
		a = adc_adj - (uint32_t)ADC->CALWORD;
		if ((a >= 3) || (a <= -3))
			a = 0;
	}
	while (i != x) {
		if (adc_dma == 2) {
			if (i == 0) {
				for (j = 1; j < 32; j++) {
					res = res + (adc_buf[j] & 0X00000fff) + a;
				}
				res = res + (adc_buf[31] & 0X00000fff) + a;
			} else {
				for (j = 0; j < 32; j++)
					res = res + (adc_buf[j] & 0X00000fff) + a;
			}
			i++;
			adc_dma = 0;
			PDMA_Trigger(PDMA_CH);
		}
	}
	ADC_STOP_CONV(adc);
	ADC->CHEN = 0;
	ADC->CHEN |= start_ch;

	res = res / y;
	return res;
}


float enthalpy_sub(float db, float wb)
{
	float db_reg, wb_reg;
	float a, reg, h;

	if (cf_flag == 1) {
		db = (db * 1.8 + 3200); // deg C-->F
		wb = (wb * 1.8 + 3200); // deg C-->F
	}
	/* 以實際代小數點一位下去計算,所以要除以10,EXP:595/10=59.5 */
	db_reg = (float)db / 100;
	wb_reg = (float)wb / 100;
	a = exp(-1.0440397 * 10000 / (wb_reg + 459.67) +
		-1.129465 * 10 + -2.7022355 * 0.01 * (wb_reg + 459.67) +
		1.289036 * 0.00001 * (wb_reg + 459.67) * (wb_reg + 459.67) +
		-2.4780681 * 0.000000001 * (wb_reg + 459.67)
		* (wb_reg + 459.67) * (wb_reg + 459.67) +
		6.5459673 * 1 * log(wb_reg + 459.67));
	h = 0.24 * db_reg + ((((0.62198 * a / (14.696 - a)) *
			       (1093 - 0.556 * wb_reg)) -
			      0.24 * (db_reg - wb_reg)) /
			     (1093 + 0.444 * db_reg - wb_reg)) * (1061 + 0.444 * db_reg);
	if (cf_flag == 1) {
		/* 0.01 [KJ/KG] v114 (h[Btu/lbm]*2.3259)-17.88444 = h[KJ/KG](Me) */
		reg = h * 100 / 0.429922613;
		reg = reg - 1788.444;
	} else {
		reg = h * 100;
	}

	return reg;
}



void SavePinSetting(void)
{
	_Pin_Setting[0] = SYS->PA_H_MFP;
	_Pin_Setting[1] = SYS->PA_L_MFP;
	_Pin_Setting[2] = SYS->PB_H_MFP;
	_Pin_Setting[3] = SYS->PB_L_MFP;
	_Pin_Setting[4] = SYS->PC_H_MFP;
	_Pin_Setting[5] = SYS->PC_L_MFP;
	_Pin_Setting[6] = SYS->PD_H_MFP;
	_Pin_Setting[7] = SYS->PD_L_MFP;
	_Pin_Setting[8] = SYS->PE_H_MFP;
	_Pin_Setting[9] = SYS->PE_L_MFP;
	_Pin_Setting[10] = SYS->PF_L_MFP;



	_PMD_Setting[0] = PA->PMD;
	_PMD_Setting[1] = PB->PMD;
	_PMD_Setting[2] = PC->PMD;
	_PMD_Setting[3] = PD->PMD;
	_PMD_Setting[4] = PE->PMD;
	_PMD_Setting[5] = PF->PMD;

	_OFFD_Setting[0] = PA->OFFD;
	_OFFD_Setting[1] = PB->OFFD;
	_OFFD_Setting[2] = PC->OFFD;
	_OFFD_Setting[3] = PD->OFFD;
	_OFFD_Setting[4] = PE->OFFD;
	_OFFD_Setting[5] = PF->OFFD;

	_PullUp_Setting[0] = PA->PUEN;
	_PullUp_Setting[1] = PB->PUEN;
	_PullUp_Setting[2] = PC->PUEN;
	_PullUp_Setting[3] = PD->PUEN;
	_PullUp_Setting[4] = PE->PUEN;
	_PullUp_Setting[5] = PF->PUEN;


	_OUT_Setting[0] = PA->DOUT;
	_OUT_Setting[1] = PB->DOUT;
	_OUT_Setting[2] = PC->DOUT;
	_OUT_Setting[3] = PD->DOUT;
	_OUT_Setting[4] = PE->DOUT;
	_OUT_Setting[5] = PF->DOUT;

}


//-------------------------------------------------------------------------------------------//
// Restore Pin Setting
//-------------------------------------------------------------------------------------------//
void RestorePinSetting(void)
{
	//uint8_t i;

	SYS_UnlockReg();

	SYS->PA_H_MFP = _Pin_Setting[0];
	SYS->PA_L_MFP = _Pin_Setting[1];
	SYS->PB_H_MFP = _Pin_Setting[2];
	SYS->PB_L_MFP = _Pin_Setting[3];
	SYS->PC_H_MFP = _Pin_Setting[4];
	SYS->PC_L_MFP = _Pin_Setting[5];
	SYS->PD_H_MFP = _Pin_Setting[6];
	SYS->PD_L_MFP = _Pin_Setting[7];
	SYS->PE_H_MFP = _Pin_Setting[8];
	SYS->PE_L_MFP = _Pin_Setting[9];
	SYS->PF_L_MFP = _Pin_Setting[10];

	SYS->Int_VREFCTL |= (SYS_VREFCTL_EXT_MODE_Msk | SYS_VREFCTL_SEL25_Msk |
			     SYS_VREFCTL_REG_EN_Msk | SYS_VREFCTL_BGP_EN_Msk);
	SYS->LDO_CTL = 0x00000B04;

	WDT->CTL = WDT_TIMEOUT_2POW18 | WDT_CTL_WTE_Msk | WDT_CTL_WTRE_Msk;


	SYS_LockReg();

	PA->PMD = _PMD_Setting[0];
	PB->PMD = _PMD_Setting[1];
	PC->PMD = _PMD_Setting[2];
	PD->PMD = _PMD_Setting[3];
	PE->PMD = _PMD_Setting[4];
	PF->PMD = _PMD_Setting[5];

	PA->OFFD = _OFFD_Setting[0];
	PB->OFFD = _OFFD_Setting[1];
	PC->OFFD = _OFFD_Setting[2];
	PD->OFFD = _OFFD_Setting[3];
	PE->OFFD = _OFFD_Setting[4];
	PF->OFFD = _OFFD_Setting[5];

	PA->PUEN = _PullUp_Setting[0];
	PB->PUEN = _PullUp_Setting[1];
	PC->PUEN = _PullUp_Setting[2];
	PD->PUEN = _PullUp_Setting[3];
	PE->PUEN = _PullUp_Setting[4];
	PF->PUEN = _PullUp_Setting[5];

	PA->DOUT = _OUT_Setting[0];
	PB->DOUT = _OUT_Setting[1];
	PC->DOUT = _OUT_Setting[2];
	if (Data_Log_Start_Flag && Data_Log_reservation_Flag &&
	    Record_mode == 1 && Log_interval_Sec >= Log_Apo_Sec)	{
		if (GLED == 1)	{
			_OUT_Setting[3] |= BIT13;
			PD->DOUT = _OUT_Setting[3];
		} else
			PD->DOUT = _OUT_Setting[3];
	} else
		PD->DOUT = _OUT_Setting[3];

	//PD->DOUT = _OUT_Setting[3];
	PE->DOUT = _OUT_Setting[4];
	PF->DOUT = _OUT_Setting[5];

	ADC->CHEN |= ADC_CH_2_MASK; /* 預設先讀取A2(BATT) */
	ADC_POWER_ON(adc);

}





void apo_io(void)
{
	ClearFlag();
	if (Data_Log_reservation_Flag == 0 && Data_Log_Start_Flag == 0)
		pkey_mode = 0;

	Power_Off = 1;

	ble_power_off();
	SavePinSetting();

	PA->IMD = 0x00000000;   // 0x00002000 change edge to electric   0x00000000 edge
	PA->IER = 0x00080000;
	PA->ISRC |= BIT3;

	ADC->CR &= ~ADC_CR_ADEN_Msk;


	SYS_UnlockReg();

	WDT->CTL  |= WDT_CTL_WTR_Msk;
	WDT->CTL &= ~WDT_CTL_WTE_Msk;
	SYS_LockReg();


	CLK_SysTickDelay(500000);	//delay_cycles_ms(500);
	SYS_UnlockReg();

	SYS->PA_L_MFP = 0x00001000;        //4個BIT 為一個功能
	if (Data_Log_Start_Flag == 0)
		SYS->PA_H_MFP = 0x00000000;
	else
		SYS->PA_H_MFP = 0x66660000;
//	SYS->PA_H_MFP = 0x00000000;
	SYS->PB_L_MFP = 0x00000000;
	SYS->PB_H_MFP = 0x00000000;
	SYS->PC_L_MFP = 0X00000000;
	SYS->PC_H_MFP = 0x00000000;
	SYS->PD_L_MFP = 0x00000000;
	SYS->PD_H_MFP = 0x00000000;
	SYS->PE_L_MFP = 0x00000000;
	SYS->PE_H_MFP = 0x00000000;
	SYS->PF_L_MFP = 0x00000000;

	PA->OFFD  = 0x00000000;//0x10700000;   //         //設"1"為不是I/O 功能
	PB->OFFD  = 0x00000000;
	PC->OFFD  = 0x00000000;
	PD->OFFD  = 0x00000000;
	PE->OFFD  = 0x00000000;
	PF->OFFD  = 0x00000000;

	//輸入,輸出 設定
	PA->PMD   =
		0x55555515; //0x50554055;//		  0x50000010;//   PA0,PA1,IN  ,key set input
	PB->PMD   = 0x55555555;
	PC->PMD   = 0X55555555;
	PD->PMD   = 0x55555555;
	PE->PMD   = 0X55555555;
	PF->PMD   = 0X55555555;//0X55555055;   // PF4,PF5 IN

	PA->PUEN  = 0x0000; //1070;
	PB->PUEN  = 0x0000;
	PC->PUEN  = 0x0000;
	PD->PUEN  = 0x0000;
	PE->PUEN  = 0x0000;
	PF->PUEN  = 0x0000; //0x0030;       // PF4,PF5 ON


	PA->DOUT	= 0x00000000; //0X0001070;
	PD->DOUT	= 0;

	if (Data_Log_Start_Flag)	{
		PB->DOUT	= 0x00001000; //0x00001000;//
		PC->DOUT	= 0x00000003; //0x00004200;//PC14;
	} else {
		PB->DOUT	= 0x00000000; //0x00001000;//
		PC->DOUT	= 0x00004000; //0x00004200;//PC14;
	}

	PE->DOUT	= 0;
	PF->DOUT	= 0;


	//disable 2.5 internal VREF and output
	SYS->Int_VREFCTL &= ~(SYS_VREFCTL_EXT_MODE_Msk | SYS_VREFCTL_SEL25_Msk |
			      SYS_VREFCTL_REG_EN_Msk | SYS_VREFCTL_BGP_EN_Msk);



	CLK_SysTickDelay(10);
	SYS->IRCTRIMCTL = 0X000000F2;

	RestorePinSetting();
	ble_power_on();

	if (StopDataLog_Flag == 0)
		Devie_Default();
	else
		VB_EN = 	0;
}

void read_RTC(void)//存取rtc暫存器
{
	//S_RTC_TIME_DATA_T sInitTime2;

	calendar_Y_M_D = RTC->CLR; //年、月、日，暫存器讀值
	time_H_M_S = RTC->TLR;//時、分、秒，暫存器讀值

	RTC_data[5] = ((calendar_Y_M_D & 0xf00000) >> 20) * 10 + ((
				calendar_Y_M_D & 0xf0000) >> 16); //RTC_Year
	RTC_data[4] = ((calendar_Y_M_D & 0x1000) >> 12) * 10 + ((
				calendar_Y_M_D & 0xf00) >> 8); //RTC_Month
	RTC_data[3] = (((calendar_Y_M_D & 0x30) >> 4) * 10) + ((
				calendar_Y_M_D) & 0xf); //RTC_Day

	RTC_data[2] = ((time_H_M_S & 0x300000) >> 20) * 10 + ((time_H_M_S & 0xf0000) >>
			16);//RTC_Hour
	RTC_data[1] = ((time_H_M_S & 0x7000) >> 12) * 10 + ((time_H_M_S & 0xf00) >>
			8); //RTC_Min
	RTC_data[0] = (((time_H_M_S & 0x70) >> 4) * 10) + (time_H_M_S & 0xf); //RTC_Sec

	reservation_Y_M_D = RTC->CAR; //年、月、日，暫存器讀值
	reservation_H_M_S = RTC->TAR;//時、分、秒，暫存器讀值

	Present_time[5] = ((reservation_Y_M_D & 0xf00000) >> 20) * 10 + ((
				  reservation_Y_M_D & 0xf0000) >> 16); //RTC_Year
	Present_time[4] = ((reservation_Y_M_D & 0x1000) >> 12) * 10 + ((
				  reservation_Y_M_D & 0xf00) >> 8); //RTC_Month
	Present_time[3] = (((reservation_Y_M_D & 0x30) >> 4) * 10) + ((
				  reservation_Y_M_D) & 0xf); //RTC_Day

	Present_time[2] = ((reservation_H_M_S & 0x300000) >> 20) * 10 + ((
				  reservation_H_M_S & 0xf0000) >> 16); //RTC_Hour
	Present_time[1] = ((reservation_H_M_S & 0x7000) >> 12) * 10 + ((
				  reservation_H_M_S & 0xf00) >> 8); //RTC_Min
	Present_time[0] = (((reservation_H_M_S & 0x70) >> 4) * 10) +
			  (reservation_H_M_S & 0xf);//RTC_Sec

}

void Rtc_Set(void)
{


	RTC->INIR = RTC_INIT_KEY;//RTC初始化 //2016/09/22
	if (RTC->INIR != 0x1) {
		RTC->INIR = RTC_INIT_KEY;
		while (RTC->INIR != 0x1);
	}

	RTC->AER = RTC_WRITE_KEY;
	while (!(RTC->AER & RTC_AER_ENF_Msk));

	RTC->AER = RTC_WRITE_KEY;//執行此程式後約15ms可以讀寫RTC
	while (!(RTC->AER & RTC_AER_ENF_Msk))
		RTC->AER = RTC_WRITE_KEY;
	RTC->TSSR |= RTC_TSSR_24H_12H_Msk;

	ymd    = (day_time[5]) << 16;//ori:0
	ymd   |= (day_time[4]) << 8;//ori:1
	ymd   |= (day_time[3]) ;//ori:2
	RTC->AER = RTC_WRITE_KEY;
	while (!(RTC->AER & RTC_AER_ENF_Msk));
	RTC->CLR = ymd;

	hms    = (day_time[2]) << 16;//ori:3
	hms   |= (day_time[1]) << 8;//ori:4
	hms   |= (day_time[0]) ;//ori:5
	RTC->AER = RTC_WRITE_KEY;
	while (!(RTC->AER & RTC_AER_ENF_Msk));
	RTC->TLR = hms;

	RTC_SetTickPeriod(RTC_TICK_1_SEC);



}

void Rtc_Alarm_Set(void)
{


	RTC->INIR = RTC_INIT_KEY;//RTC初始化
	if (RTC->INIR != 0x1) {
		RTC->INIR = RTC_INIT_KEY;
		while (RTC->INIR != 0x1);
	}
	RTC->AER = RTC_WRITE_KEY;//執行此程式後約15ms可以讀寫RTC
	while (!(RTC->AER & RTC_AER_ENF_Msk))
		RTC->AER = RTC_WRITE_KEY;
	RTC->TSSR |= RTC_TSSR_24H_12H_Msk;

	reserved_ymd = (Start_time[5]) << 16;
	reserved_ymd |= (Start_time[4]) << 8;
	reserved_ymd |= (Start_time[3]) ;
	RTC->AER = RTC_WRITE_KEY;
	while (!(RTC->AER & RTC_AER_ENF_Msk));
	RTC->CAR = reserved_ymd;

	reserved_hms = (Start_time[2]) << 16;
	reserved_hms |= (Start_time[1]) << 8;
	reserved_hms |= (Start_time[0]) ;
	RTC->AER = RTC_WRITE_KEY;
	while (!(RTC->AER & RTC_AER_ENF_Msk));
	RTC->TAR = reserved_hms;

	/* Enable RTC Alarm Interrupt */
	RTC->RIIR = 0x1;//clear flag
	RTC->RIER |= 1;
	NVIC_EnableIRQ(RTC_IRQn);

}

void delay_cycles(uint32_t x)
{
	uint32_t j;
	for (j = 0; j <= x; j++) {
		__NOP();
	}

}



void PDMA_Init(void)
{

	PDMA_T *pdma;

	PDMAGCR->GCRCSR |= ((1 << PDMA_CH) << 8);
	pdma = (PDMA_T *)((uint32_t) PDMA0_BASE + (0x100 * PDMA_CH));
	pdma->CSR = (pdma->CSR & ~PDMA_CSR_APB_TWS_Msk) | PDMA_WIDTH_32;
	pdma->BCR = (ADC_BUF_COUNT << 2);
	pdma->SAR = (uint32_t)&ADC->PDMA;
	pdma->DAR = (uint32_t)adc_buf;
	pdma->CSR = (pdma->CSR & ~(PDMA_CSR_SAD_SEL_Msk | PDMA_CSR_DAD_SEL_Msk)) |
		    (PDMA_SAR_FIX | PDMA_DAR_INC);
	pdma->TCR = (pdma->TCR & ~PDMA_TCR_PDMA_TCR_Msk) | 0x5555;
	pdma->CSR = (pdma->CSR & ~PDMA_CSR_TO_EN_Msk) | (0 << PDMA_CSR_TO_EN_Pos);
	pdma->IER |= PDMA_IER_TD_IE_Msk;
	PDMAGCR->DSSR0 = (PDMAGCR->DSSR0 & ~DMA_GCR_DSSR0_CH1_SEL_Msk) |
			 (PDMA_ADC << DMA_GCR_DSSR0_CH1_SEL_Pos);
	if (PDMA_ADC == PDMA_MEM)
		pdma->CSR &= ~PDMA_CSR_MODE_SEL_Msk;
	else if (PDMA_ADC & 0x10)
		pdma->CSR = (pdma->CSR & ~PDMA_CSR_MODE_SEL_Msk) | 0x4; /* IP to memory */
	else
		pdma->CSR = (pdma->CSR & ~PDMA_CSR_MODE_SEL_Msk) | 0x8; /* memory to IP */
	NVIC_EnableIRQ(PDMA_IRQn);

}

void PDMA_IRQHandler(void)
{

	uint32_t status = PDMA_GET_INT_STATUS();

	if (status == 2) {
		adc_dma = status;
		PDMA_CLR_CH_INT_FLAG(1, PDMA_ISR_TD_IS_Msk);
	}
	ble_pdma_handler(status);

}


/* #define P_OFF_TIME            100000            approximate 2 sec
 *#define P_ON_TIME             50000
 */
void EINT1_IRQHandler(void)
{
	int32_t i = 0;
	int32_t key_tt;
	uint8_t a = 0;

	if (PA->ISRC & PWR_WAKE) {
		APOcounter = 0;
		PA->ISRC |= PWR_WAKE;

		if (pkey_mode == 1)
			key_tt = P_OFF_TIME;
		else
			key_tt = P_ON_TIME;
		do {
			if (KEY_PWR == 0)
				a = 1;
			i++;
		} while (a == 0 && i < key_tt);

		if (pkey_mode == 0) {
			if (i >= P_ON_TIME) {            // Pressing time > P_ON_TIME, power on
				Apo_Exit_Flag = 1;
				Power_On_Flag = 1;
				Wake_Log_Flag = 0;
				Flash_write_flag = 0;
				Data_Log_Start_Flag = 0;
				Data_Log_reservation_Flag = 0;
				Record_mode = 0;
				GLED = 0;
				led_mode = 0;
			}
		} else {
			if (i >= P_OFF_TIME) {

				if (led_mode != 10) {
					led_mode = 10;
					TIMER3->CMPR   = 1638 - 1;
					TIMER3->DR =  0;
					TIMER3->CTL    |= TIMER_CTL_TMR_EN_Msk;//NVIC_EnableIRQ(TMR3_IRQn);
					led_count1 = 0;
					led_count = 0;
					RLED = 0;
					BLED = 0;
					GLED = 1;
				}

				StopDataLog_Flag = 1;
				VB_EN = 	1;
				Apo_Exit_Flag = 1;

				RTC->RIER &= ~BIT0;

				Record_mode = 0;
				Data_Log_Start_Flag = 0;
				Data_Log_reservation_Flag = 0;
				//CLK_SysTickDelay(50000);
				Power_Off = 1;
			} else {
				//power_off_flag=0;
				if (cali_flag == 1 || cali_flag == 2) {
					__NOP();//adj_up =1;

				} else {
					if (i <= P_TWICE_TIME)
						__NOP();//atm_flag=1;
					else {
						if (apo_select == 0 && Data_Log_reservation_Flag == 0)
							key_cont++;//key_st=0;//
					}

				}

			}
		}
	}

}

void RTC_IRQHandler(void)
{

	if ((RTC->RIER & RTC_RIER_TIER_Msk) &&
	    (RTC->RIIR & RTC_RIIR_TIF_Msk)) {        /* tick interrupt occurred */
		RTC->RIIR = 0x2;
		Wake_up_cont++;
		if (Wake_up_cont >=
		    Log_interval_Sec) {	//由interval時間來決定多久 由RTC喚醒MCU一次
			Wake_up_cont = 0;
			if (Data_Log_Start_Flag &&
			    Data_Log_reservation_Flag) { //當DATA LOG設定完成後及ALM動作才會有喚醒
				APOcounter = 0;
				Flash_write_flag = 1;

				if (!Power_On_Flag) {
				#if(LOG_APO==1)
					Apo_Exit_Flag = 1;				//離開 APO用的旗標
					Power_On_Flag = 1;				//
					Wake_Log_Flag = 1;				//自動喚醒進行LOG的旗標
				#endif
//						Apo_Exit_Flag=1;				//開?自動關機,此旗標要打開			//離開 APO用的旗標
//						Power_On_Flag=1;				//開?自動關機,此旗標要打開			//離開 APO用的旗標
//						Wake_Log_Flag=1;				//開?自動關機,此旗標要打開			//離開 APO用的旗標
					__NOP();
				}
			}
		}


		if (Data_Log_reservation_Flag) {
			duration_cont++;
			if((duration_cont > (Duration_Min * 60) || (log_addr >= DL_Address))) {
				duration_cont = 0;
				Data_Log_Start_Flag = 0;
				Data_Log_reservation_Flag = 0;
				pkey_mode = 0;
				Record_mode = 0;
				ClearFlag();
				Power_Off = 1;
				//apo_io();

			}

		}

	}
	if ((RTC->RIER & RTC_RIER_AIER_Msk) && (RTC->RIIR & RTC_RIIR_AIF_Msk)) {
		/* alarm interrupt occurred */
		RTC->RIIR = 0x1;//clear flag

		Data_Log_Start_Flag = 1;
		Data_Log_reservation_Flag = 1;
		Wake_up_cont = 0;
		if (Record_mode == 1 && Log_interval_Sec >= Log_Apo_Sec) {
			//Power_Off =1;
#if(LOG_APO==1)
			Power_On_Flag = 0; //開?自動關機,此旗標要打開			//離開 APO用的旗標
			Wake_Log_Flag = 1; //開?自動關機,此旗標要打開			//離開 APO用的旗標
#endif
//			Power_On_Flag=0;//開?自動關機,此旗標要打開			//離開 APO用的旗標
//			Wake_Log_Flag=1;//開?自動關機,此旗標要打開			//離開 APO用的旗標
			TIMER2->CTL    |= TIMER_CTL_WAKE_EN_Msk ;
			TIMER3->CTL    |= TIMER_CTL_WAKE_EN_Msk ;
		}
	}



}

static void trigger_apo_blink(void)
{
	if (apo_flag) {
		led_mode = 3;
		TIMER2->CMPR = 16384 - 1;
		TIMER2->DR = 0;
	} else {
		led_mode = 2;
		TIMER2->CMPR = 32768 - 1;
		TIMER2->DR = 0;
		TIMER3->CMPR = 5461 - 1;
		TIMER3->DR = 0;
		TIMER3->CTL |= TIMER_CTL_TMR_EN_Msk;
	}
	apo_select = 1;
	GLED = 0;
	RLED = 0;
	BLED = 0;
	led_count = 0;
}

/*
 * @brief: 暫存按鍵狀態且處理按鍵行為
 * @input:
 *    id: 按鍵代碼，power, enter (mode), combo (all)
 *    *key: 按鍵結構
 * @return: N/A
 */
static void process_button_event(enum button_id id, struct KEY_STRUCT *key)
{
	if (key->key_action_status == KEY_NONE || key->hold_lock_flag)
		return;

	button_pending.status = (enum button_status)key->key_action_status;
	button_pending.id = id;
	button_pending.duration_ms = key->key_hold_cnt * 100;
	button_pending.ts_hour = key->key_rtc_time[0];
	button_pending.ts_min = key->key_rtc_time[1];
	button_pending.ts_sec = key->key_rtc_time[2];

	if (id == BUTTON_ID_POWER &&
	    button_pending.status == BUTTON_DOUBLE_CLICK) {
		apo_flag ^= 1;
		trigger_apo_blink();
	}

	if (button_pending.status == BUTTON_HELD)
		key->hold_lock_flag = 1;

	key->key_action_status = KEY_NONE;
}

void TMR0_IRQHandler(void) /* 100 ms int */
{
	TIMER_ClearIntFlag(TIMER0);
	ble_tick_100ms();

	Key_Scan();

	process_button_event(BUTTON_ID_ALL, &combo_key);
	process_button_event(BUTTON_ID_POWER, &power_key);
	process_button_event(BUTTON_ID_MODE, &enter_key);

	if (apo_flag) {
		APOcounter++;
		if (APOcounter >= APO_TIME)
			Power_Off = 1;
	} else {
		APOcounter = 0;
	}
}



void TMR1_IRQHandler(void)
{
	TIMER_ClearIntFlag(TIMER1);

	ad_cont++;
	if (ad_cont >= ad_cont_set) {
		ad_read = 1;
		ad_cont = 0;
	}
}



void TMR2_IRQHandler(void)    // 1 int
{

	TIMER_ClearIntFlag(TIMER2);

	read_RTC();
	switch (led_mode) {
	case 1://Power On, Normal Mode
		if (ble_ctx.state == BLE_STATE_CONNECTED ||
		    ble_ctx.state == BLE_STATE_MODULE_OTA) {
			BLED = 0;
			GLED = 1;
			RLED = 0;
			led_mode = 5;
			break;
		}

		led_count++;
		if (led_count >= 2) {
			led_count = 0;
			TIMER3->CMPR   = 8192 - 1;									////每秒 4次
			TIMER3->DR =  0;
			TIMER3->CTL    |= TIMER_CTL_TMR_EN_Msk;//NVIC_EnableIRQ(TMR3_IRQn);
			led_count1 = 0;
			if (batt_led_flag == 0) {
				if (cali_flag == 1) {
					BLED = 1;
					GLED = 0;
					RLED = 0;
				} else {
					BLED = 0;
					GLED = 1;
					RLED = 0;
				}
			} else
				RLED = 1;
			if (apo_select)
				apo_select = 0;
		}
		break;
	case 2: //  Disabled APO
		led_count++;
		if (led_count >= 2) {
			//NVIC_EnableIRQ(EINT1_IRQn);
			TIMER2->CMPR   = 32768 - 1;
			TIMER2->DR =  0;
			NVIC_EnableIRQ(TMR2_IRQn);
			RLED = 0;
			led_count1 = 0;
			TIMER3->CMPR   = 8192 - 1;									////每秒 4次
			TIMER3->DR =  0;
			TIMER3->CTL    |= TIMER_CTL_TMR_EN_Msk;//NVIC_EnableIRQ(TMR3_IRQn);
			if (batt_led_flag == 0)
				GLED = 1;
			else
				RLED = 1;
			led_mode = 1;
		}
		break;
	case 3: //Enabled APO
		led_count++;
		if (led_count == 1)
			RLED = 1;

		if (led_count >= 4) {
			//NVIC_EnableIRQ(EINT1_IRQn);
			TIMER2->CMPR   = 32768 - 1;
			TIMER2->DR =  0;
			NVIC_EnableIRQ(TMR2_IRQn);
			RLED = 0;
			led_count1 = 0;
			TIMER3->CMPR   = 8192 - 1;									////每秒 4次
			TIMER3->DR =  0;
			TIMER3->CTL    |= TIMER_CTL_TMR_EN_Msk;//NVIC_EnableIRQ(TMR3_IRQn);
			if (batt_led_flag == 0)
				GLED = 1;
			else
				RLED = 1;
			led_mode = 1;
		}
		break;
	case 4: //Low Battery
		led_count++;
		if (led_count >= 2) {
			led_count = 0;
			TIMER3->CMPR   = 8192 - 1;									////每秒 4次
			TIMER3->DR =  0;
			TIMER3->CTL    |= TIMER_CTL_TMR_EN_Msk;//NVIC_EnableIRQ(TMR3_IRQn);
			led_count1 = 0;
			RLED = 1;
		}
		break;
	case 5: //Awaiting Data Log Settings
		GLED = 1;
		if (ble_ctx.state == BLE_STATE_PERIOD_SYNC) {
			led_mode = 1;
			if (sn_written) {
				Power_Off = 1;
				sn_written = 0;
			}
		}
		break;
	case 7: //  Data Log Scheduled and Waiting
		led_count++;
		if (led_count >= 2) {
			led_count = 0;
			TIMER3->CMPR   = 8192 - 1;									////每秒 4次
			TIMER3->DR =  0;
			TIMER3->CTL    |= TIMER_CTL_TMR_EN_Msk;//NVIC_EnableIRQ(TMR3_IRQn);
			led_count1 = 0;
			BLED = 1;
			GLED = 0;
			RLED = 0;
		}
		break;
	case 8://Logging Data
		led_count++;
		if (led_count >= 5) {
			led_count = 0;
			TIMER3->CMPR   = 8192 - 1;									////每秒 4次
			TIMER3->DR =  0;
			TIMER3->CTL    |= TIMER_CTL_TMR_EN_Msk;//NVIC_EnableIRQ(TMR3_IRQn);
			led_count1 = 0;
			BLED = 0;
			GLED = 1;
			RLED = 0;
		}
		break;
	case 10:
		led_count++;
		if (led_count >= 2) {
			led_count = 0;
			//led_mode=1;
			GLED = 1;
		}
		break;
		//--------------
	}

}

void TMR3_IRQHandler(void)
{
	TIMER_ClearIntFlag(TIMER3);
	switch (led_mode) {
	case 1:
		led_count1++;
		if (led_count1 >= 1) {
			TIMER3->CTL    &= ~TIMER_CTL_TMR_EN_Msk;//NVIC_DisableIRQ(TMR3_IRQn);
			led_count1 = 0;
			if (batt_led_flag == 1)
				RLED = 0;

			if (cali_flag == 1) {
				GLED = 0;
				BLED = 0;
				RLED = 0;
			} else {
				GLED = 0;
				if (indicator == INDI_BLUE)
					BLED = 1;
				else
					RLED = 1;
			}
		}
		break;
	case 2:
		led_count1++;
		if (led_count1 & BIT0) {
			RLED = 1;
		} else {
			RLED = 0;
		}
		break;
	case 4:
		led_count1++;
		if (led_count1 >= 1) {
			TIMER3->CTL    &= ~TIMER_CTL_TMR_EN_Msk;//NVIC_DisableIRQ(TMR3_IRQn);
			led_count1 = 0;
			RLED = 0;
		}
		break;
	case 6:
		led_count1++;
		if (led_count1 & BIT0) {
			GLED = 1;
		} else {
			GLED = 0;
		}
		break;
	case 7:
		led_count1++;
		if (led_count1 >= 1) {
			TIMER3->CTL    &= ~TIMER_CTL_TMR_EN_Msk;//NVIC_DisableIRQ(TMR3_IRQn);
			led_count1 = 0;
			BLED = 0;
		}
		break;
	case 8:
		led_count1++;
		if (led_count1 >= 1) {
			TIMER3->CTL    &= ~TIMER_CTL_TMR_EN_Msk;//NVIC_DisableIRQ(TMR3_IRQn);
			led_count1 = 0;
			GLED = 0;
			if (indicator == INDI_BLUE)
				BLED = 1;
			else
				RLED = 1;
		}
		break;
	case 9:
		led_count1++;
		if (led_count1 & BIT0) {
			GLED = 1;
			RLED = 1;
		} else {
			GLED = 0;
			RLED = 0;
		}
		break;
	case 10:
		GLED ^= 1;

		break;
	}
}

void UART0_IRQHandler(void)//UART1_IRQHandler
{

	if (UART0->ISR & (UART_ISR_RTO_IS_Msk | UART_ISR_RDA_IS_Msk)) {
		while ((UART0->FSR & UART_FSR_RX_EMPTY_F_Msk) == 0)
			rxd_data[rxd_cont] = UART0->RBR;
		//rxd_cont++;
		//if((rxd_data[rxd_cont]==0x0a && rxd_data[rxd_cont-1]==0x0d)&& rxd_cont>=rxd_data[1]-1)
		if ((rxd_data[rxd_cont - 1] == 'E') && cali_flag == 2) {
			//uart_rxd_flag=1;
			rxd_cont = 0;
		} else if ((rxd_data[rxd_cont] == 0x0a && rxd_data[rxd_cont - 1] == 0x0d) &&
			   cali_flag == 1) { //
			rxd_cont = 0;
		} else {
			rxd_cont++;
			if (cali_flag == 1) {
				if (rxd_data[0] != '%')
					rxd_cont = 0;
			}

		}
//
		if (cali_flag == 1) {
			if (rxd_cont >= 14) {
				rxd_cont = 0;
			}
		} else {
			if (rxd_cont >= 6) {
				rxd_cont = 0;
			}
		}

	}

}

void UART1_IRQHandler(void)
{
	ble_uart_rx_handler();
}

void read_fmc1(uint8_t x, uint8_t y, uint32_t addr)
{
	volatile uint8_t i = 0;
	volatile uint32_t  *a;
	volatile float calit_r[1];

	SYS_UnlockReg();
	//FMC_Open();
	FMC->ISPCON |=  FMC_ISPCON_ISPEN_Msk;
	FMC->ISPCMD = FMC_ISPCMD_READ;
	a = 0;
	if (x == 1) {
		if (y == 1) { //讀取序號和校正日期等
			for (i = 0; i < 14; i++) {
				FMC->ISPADR = addr + i * 4; // 丟位置
				FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
				while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);
				cali_value_st[i] = FMC->ISPDAT; // 取值
			}
		}
		if (y == 0) {
			FMC->ISPADR = addr;
			FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
			while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);
			value[0] = FMC->ISPDAT;
			FMC->ISPADR = addr + 4;
			FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
			while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);
			value[1] = FMC->ISPDAT;

		}
		if (y == 2) {
			FMC->ISPADR = addr ;
			FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
			while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);
			cali_value_st[0] = FMC->ISPDAT;
		}
	} else {
		if (y == 0) {
			FMC->ISPADR = addr;
			FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
			while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);
			a = (uint32_t *)calit_r;			//以浮點型態讀取	,變數需為浮點
			*a = FMC->ISPDAT;						//以浮點型態讀取	,變數需為浮點
			value[0] = calit_r[0];	//以浮點型態讀取	,變數需為浮點

			FMC->ISPADR = addr + 4;
			FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
			while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);
			a = (uint32_t *)calit_r;			//以浮點型態讀取	,變數需為浮點
			*a = FMC->ISPDAT;						//以浮點型態讀取	,變數需為浮點
			value[1] = calit_r[0];	//以浮點型態讀取	,變數需為浮點

		}
		if (y == 1) {
			for (i = 0; i < 4; i++) {
				FMC->ISPADR = addr + i * 4;
				FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
				while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);
				a = (uint32_t *)calit_r;
				*a = FMC->ISPDAT;
				valueRH[i] = calit_r[0];
				a++;
			}
		}
	}
	// FMC_Close();
	FMC->ISPCON &= ~FMC_ISPCON_ISPEN_Msk;
	SYS_LockReg();

}

void write_fmc1(uint8_t x, uint8_t y, uint32_t addr)
{
	volatile uint8_t i = 0;
	volatile uint32_t  *a;
	volatile float calit_w[1];

	SYS_UnlockReg();
	FMC->ISPCON |=  FMC_ISPCON_ISPEN_Msk;

	FMC->ISPCMD = FMC_ISPCMD_PAGE_ERASE;
	a = 0;
	if (x == 1) { //寫入校正日期和AA用 ; NTC OFFSET ; ADC OFFSET
		FMC->ISPADR = addr;
		FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
		while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) ;
		FMC->ISPCMD = FMC_ISPCMD_WRITE;
		if (y == 1) {
			while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);
			for (i = 0; i < 14; i++) {
				FMC->ISPADR = addr + i * 4;
				FMC->ISPDAT = cali_value_st[i];
				FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
				while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);
			}
		}
		if (y == 0) {
			FMC->ISPADR = addr;
			FMC->ISPDAT = value[0];//ADC->CALWORD;
			FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
			while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);

			FMC->ISPADR =	addr + 4;
			FMC->ISPDAT =	value[1];//ntc_offset;
			FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
			while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);

		}
	} else {		 //寫入NTC OFFSET校正值和ADC OFFSET
		FMC->ISPADR = addr;
		FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
		while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) ;
		FMC->ISPCMD = FMC_ISPCMD_WRITE;
		if (y == 0) {	//Ntc offset and ADC calibration value
			FMC->ISPADR = addr;
			calit_w[0] = value[0]; //ADC->CALWORD;		//以浮點型態存取	,變數需為浮點
			a = (uint32_t *)calit_w; 			//以浮點型態存取
			FMC->ISPDAT = *a; 					//以浮點型態存取
			FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
			while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);

			FMC->ISPADR = addr + 4;
			calit_w[0] = value[1]; //(float)ntc_offset;		//以浮點型態存取	,變數需為浮點
			a = (uint32_t *)calit_w; 			//以浮點型態存取
			FMC->ISPDAT = *a; 					//以浮點型態存取
			FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
			while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);


		}
		if (y == 1) {
			for (i = 0; i < 4; i++) { //寫入預設的濕度校正值
				FMC->ISPADR = addr + i * 4;
				calit_w[0] = valueRH[i];
				a = (uint32_t *)calit_w;
				FMC->ISPDAT = *a;
				FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
				while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);
			}

		}
	}
	//FMC_Close();
	FMC->ISPCON &= ~FMC_ISPCON_ISPEN_Msk;
	SYS_LockReg();

}

//------------------------------------------------------------------------------------------------------------------------//
//read 24LC02 (NTC1)
//------------------------------------------------------------------------------------------------------------------------//
float eep_rword(uint8_t addr)
{
	uint32_t i;
	uint32_t  *a;
	float calit_r[1], j;

	I2c_StartCondition();
	I2c_WriteByte(0xA0);
	I2c_WriteByte(addr * 4);
	I2c_StartCondition();
	I2c_WriteByte(0xA1);

	eerom_buf[0] = I2c_ReadByte();
	eerom_buf[1] = I2c_ReadByte();
	eerom_buf[2] = I2c_ReadByte();
	eerom_buf[3] = I2c_ReadByte_end();

	I2c_StopCondition();


	i = eerom_buf[3];
	i = i << 8;
	i |= eerom_buf[2];
	i = i << 8;
	i |= eerom_buf[1];
	i = i << 8;
	i |= eerom_buf[0];

	a = (uint32_t *)calit_r;
	*a = i;
	j = calit_r[0];

	return j;
}


//------------------------------------------------------------------------------------------------------------------------//
//Write 24LC02 (NTC1)
//------------------------------------------------------------------------------------------------------------------------//
void eep_wword(uint8_t addr, float data)
{
	uint32_t i, j;
	uint32_t  *a;

	float calit_w[1];

	calit_w[0] = data;
	a = (uint32_t *)calit_w;
	j = *a;

	i = j & 0x000000ff;
	eerom_buf[0] = i;
	j = j >> 8;
	i = j & 0x000000ff;
	eerom_buf[1] = i;
	j = j >> 8;
	i = j & 0x000000ff;
	eerom_buf[2] = i;
	j = j >> 8;
	i = j & 0x000000ff;
	eerom_buf[3] = i;     ////

	I2c_StartCondition();
	I2c_WriteByte(0xa0);
	I2c_WriteByte(addr * 4);
	I2c_WriteByte(eerom_buf[0]);
	I2c_WriteByte(eerom_buf[1]);
	I2c_WriteByte(eerom_buf[2]);
	I2c_WriteByte(eerom_buf[3]);
	I2c_StopCondition();
	delay_cycles(1000);
}


void power_off_status(void)
{
	if (power_key.hold_lock_flag == 0) {
	
		if (power_key.key_hold_cnt >= 19) {
		
			if (led_mode != 10) {
				led_mode = 10;
				TIMER3->CMPR = 1638 - 1;
				TIMER3->DR =  0;
				TIMER3->CTL |= TIMER_CTL_TMR_EN_Msk;
				led_count1 = 0;
				led_count = 0;
				RLED = 0;
				BLED = 0;
				GLED = 1;
			}

			StopDataLog_Flag = 1;
			VB_EN = 1;
			Apo_Exit_Flag = 1;

			RTC->RIER &= ~BIT0;

			Record_mode = 0;
			Data_Log_Start_Flag = 0;
			Data_Log_reservation_Flag = 0;
			Power_Off = 1;

			uint8_t exit = 0;

			while (!KEY_PWR & !exit) {
				/* 按過5秒直接關機 */
				if (power_key.key_hold_cnt >= 50)
					exit = 1;
			}
		}
	}

	if (Power_Off) {
		NVIC_DisableIRQ(TMR0_IRQn);
		NVIC_DisableIRQ(TMR1_IRQn);
		NVIC_DisableIRQ(TMR2_IRQn);
		NVIC_DisableIRQ(TMR3_IRQn);
		RLED = 0;
		BLED = 0;
		GLED = 0;
		VB_EN = 0;
		Power_Off = 0;
	}
}

/*

依目前Log_Data_Address_Calculate()此作法存在1個問題,若當儲存筆數接近"DL_Address"時
筆數計數會由(DL_Address / 2)開始進行逐步檢查,有可能要確認約1040319筆資料才會找到最後筆數

暫時保留原作法,若日後客戶反應等待時間太久可採用下述2分法程式
下述方式約21次讀取Flash內容即可找到相應筆數,大幅度的降低找尋時間

 int16_t read_data(uint32_t mid)
{
	int16_t data;
	data = SpiFlash_NormalRead(mid);
	SpiFlash_WaitReady();
	
	return data;
}

uint32_t FindDataLogCount(void)
{
    uint32_t low = 0;
    uint32_t high = DL_Address;
    uint32_t mid;
    uint16_t value = 0;

    while (low < high) {
	    
	SYS_UnlockReg();
		WDT->CTL  |= WDT_CTL_WTR_Msk;
	SYS_LockReg();
	    
        mid = low + (high - low) / 2;

	value = read_data(mid);
	    
        if (value == 0xFFFF) {
		
            high = mid;
		
        } else {
		
            low = mid + 1;
        }
    }

    return low;
}


void New_Log_Data_Address_Calculate(void)
{
	Log_times_count = FindDataLogCount();

	if (Log_times_count == 0) {

		DL_Calculate_Exit = 3;

	} else  {

		Data_count = (Log_times_count - 32) % DL_data_count; //8->5
		if (Data_count != 0)
			Data_count = ((Log_times_count - 32)  / DL_data_count) + 1; //8->5
		else
			Data_count = ((Log_times_count - 32)  / DL_data_count) + 0; //8->5
	}

}
*/

