/******************************************************************************
 * @file     LCD.h
 * @brief    NANO1x2 LCD driver header file
 * @version  1.0.1
 * @date     04, September, 2013
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __RF1200_H__
#define __RF1200_H__
#include "Nano1X2Series.h"

#ifdef __cplusplus
extern "C"
{
#endif
#define KEY_PWR		PA3
#define PWR      	BIT0
#define PWR_WAKE 	BIT3
#define VB_EN	 	PB12
#define APO_ON   	PC14
#define ADJ1     	PC2
#define GLED     	PD13
#define RLED	 	PD10
#define BLED     	PD9
#define INDI_RED  	0
#define INDI_BLUE 	1

#define BATT_100	416
#define BATT_80		392
#define BATT_60		369
#define BATT_40		345
#define BATT_20		322

#define APO_TIME    	72000

struct saved_info {
	uint8_t sn[4];
	uint8_t cal_date[4];
	uint8_t hw_ver[2];
	uint8_t apo;
	uint8_t r_b;
	uint8_t reserved[2];
	uint16_t cs;
};

volatile uint16_t ver_ = 001;
void power_sub(void);                         ///< POWER ???
volatile uint8_t Power_Off = 0;                   ///< POWER off ??
volatile uint64_t APOcounter = 0;
void apo_io(void);
volatile uint8_t apo_flag = 1, Apo_Exit_Flag = 0, pkey_mode = 0; //
volatile uint8_t ver_flag = 0;
volatile int16_t ad_cont = 0; //,count=0;
volatile uint8_t ad_read = 0; //,init_flag=0;
void Device_Init(void);




/*******************************************************************************
  UP PDMA,AD相關參數定義
*******************************************************************************/
#define ADC_BUF_COUNT 								32
//PDMA CH
#define PDMA_CH            1
volatile uint32_t adc_buf[ADC_BUF_COUNT];
void PDMA_Init(void);
int32_t adc_read(uint16_t x, uint16_t y, uint32_t start_ch);
volatile uint8_t adc_dma;
volatile uint32_t adc_adj;


/*******************************************************************************
  BATT相關參數定義
*******************************************************************************/
void Battery(void);
volatile uint8_t batt_led_flag = 0;
volatile uint8_t batt_level = 0; //


/*******************************************************************************
  濕度/溫度量測相關參數定義
*******************************************************************************/
volatile int32_t Dew_T;
volatile float est1;
volatile int32_t wb;
/*寫固定的NTC(Group_J ,因固定買5種(H,I,J,K,L),寫中間TYPE 即可)*/
volatile uint8_t ntc_table = 9;
void temp(void);
void temp_avg(void);
void chk_cf(void);
volatile int32_t temp_avg_data[5];
volatile uint8_t temp_avg_cont = 0;
volatile uint8_t cf_flag = 0;
volatile int32_t ntc_offset = 0;
//volatile uint8_t err=0;
volatile int32_t T_disp, tdisp, NTC_disp;
volatile int8_t start = 1;



int32_t read_Humi(void);
void HTU21D_init(void);
void humidity(void);
volatile int32_t hmdis_a;
volatile int8_t hmdis_avg = 0;
volatile int16_t humdity_value = 0; //,RH_value=0;
float humi;
//volatile int32_t tdisp_1 = 0;
volatile int32_t rh1;
volatile int16_t Dew_T_Value = 0, Wb_Value = 0, Dry_bulb_Value = 0; //Ntc_Value=0,,disp_d,disp_2;

volatile float Enthalpy = 0;
float enthalpy_sub(float db, float wb);

/*******************************************************************************
  校正相關參數定義
*******************************************************************************/
volatile int8_t cali_flag = 0;
void cali_sub(void);

/*******************************************************************************
  UART相關參數定義
*******************************************************************************/
volatile int8_t rxd_data[30];
volatile int8_t txd_data[20];
volatile int8_t txd_cont = 0;
volatile int8_t rxd_cont = 0;

/*******************************************************************************
  FLASH相關參數定義
*******************************************************************************/
volatile int32_t value[2];
volatile float valueRH[7];

/*******************************************************************************
  24LC512/24LC02顯示轉換相關參數定義
*******************************************************************************/
volatile uint8_t eerom_buf[20];                 //24LC02 24LC256共用
void  eep_wword(uint8_t addr, float data);
float eep_rword(uint8_t addr);

/*
//--- 24LC512 ------------------------------------------------------------
//uint8_t eerom_buf[20];                 //24LC02 24LC256共用
typedef void (*I2C_FUNC)(uint32_t u32Status);
static I2C_FUNC I2C1HandlerFn = NULL;
volatile uint8_t eep256_rword_error = 0;

int16_t re2rom_data_1[7];              //re2rom_data_1[4]
volatile uint8_t eerom_DataLen;
volatile uint8_t eerom_EndFlag = 0;
volatile uint16_t eerom256_addr = 0;
void eerom256_w(uint32_t u32Status);
void eerom256_r(uint32_t u32Status);
void eep256_wword(uint16_t addr,int16_t data);
int16_t eep256_rword(uint16_t addr);
*/
//volatile uint32_t log_num;             //logging 筆數



volatile uint16_t P_TWICE_TIME	=	9500;
volatile uint32_t P_OFF_TIME		=	240000;//            100000             // approximate 2 sec
volatile uint32_t P_ON_TIME 		= 120000;
volatile uint32_t LedFlashStop	=	170000;

//volatile uint32_t testcont2			=	10000;

volatile uint8_t indicator;

volatile uint32_t cali_value_st[14];//save ID專用

volatile uint8_t Serial_Number[4] = {0x17, 0x04, 0x00, 0x02}; // yymmXXXX 產品出貨日期和產品編號 	(傳送資料為10進制,以16進制表示
volatile uint8_t Cal_Date[4]      = {0x20, 0x17, 0x04, 0x05}; // yyyymmdd	校正日期	(傳送資料為10進制,以16進制表示

volatile uint8_t _1S = 0;

//  PWM相關參數定義

#define SAMPLE_CNT 32
static uint8_t volatile cap_index;
//volatile uint16_t PWM_Duty=50;  //59
//volatile uint16_t BPWM_Duty=100;
//volatile uint16_t PWM_Freq=800;
void BLED_PWM(void);
void RLED_PWM(void);
void LED_OFF(void);
void LED_SUB(void);
volatile uint8_t ShowOneTime_Flag = 0, LowBattFlash = 0;


volatile uint8_t key_st = 0, key_cont = 0, apo_select = 0;
void apo_func(void);
volatile uint8_t batt_read_flag = 0; //,___1S=0,Flash=0,__2S=0;

//volatile uint8_t Bled_flag=0;
//volatile uint16_t BLED1S=0;
//volatile float BLEDPWM_Duty=100.0;

/* Log_Data 相關參數定義 */

#define Log_Apo_Sec 1

void Devie_Default(void);
void ClearFlag(void);
volatile int32_t key_5s_cont = 0;

/* RTC相關變數 */
#define RTC_TICK_TEST    0
void Set_default_value(void);
volatile uint8_t Set_default_Flag = 1;
void read_RTC(void);
volatile uint8_t day_time[6] = {0, 0, 0, 1, 1, 16} ;; //設定RTC的時間，格式需為BCD碼
volatile uint32_t hms;
volatile uint32_t ymd;
volatile uint8_t RTC_data[6] = {0};//放置讀取出RTC時間暫存器
volatile uint8_t  Present_time[6] = {0, 0, 0, 20, 3, 17} ; //現在的時間
volatile uint8_t Start_time[6] = {0, 0, 0, 20, 3, 17} ; //預約的時間
volatile uint32_t calendar_Y_M_D; //= 0;
volatile uint32_t time_H_M_S; //= 0;
volatile uint32_t reservation_Y_M_D; //= 0;
volatile uint32_t reservation_H_M_S; //= 0;

void Rtc_Alarm_Set(void);
void Rtc_Set(void);
volatile uint32_t reserved_hms;//預約時分秒
volatile uint32_t reserved_ymd;//預約年月日

volatile uint16_t Wake_up_cont = 0, Log_interval_Sec = 0; //logging間隔 (convert to second)
volatile uint16_t Duration_Min = 0; //持續LOG的時間,以分為單位
volatile uint32_t duration_cont = 0;	//用來計數,持續LOG時間的CONT
volatile uint8_t Record_mode = 0; //持續 DATA LOG的模式  0:ALL Function+D.L+Beacon ; 1:Only D.L
volatile uint8_t Wake_Log_Flag = 0;
volatile uint8_t Power_On_Flag = 0;

/* LOG相關變數 */
#define DL_Address    2080639      //Data log空間多預留1個Page 256 Bytes包含檔頭32 Bytes, OTA儲存檔案空間為32K
							 // (4M - (32K + 256 Bytes))/2 Bytes =  [[(4*1024*1024) - ((32 * 1024) + 256)] / 2] - 1 = 2080639

void Log_Data(void);
volatile uint8_t Flash_write_flag = 0, Flash_read_flag = 0;
volatile uint8_t Data_Log_Start_Flag = 0;
volatile uint8_t Data_Log_reservation_Flag = 0; //設定預設時間到,開始DATA LOG用的旗標,由RTC ALARM 動 作時設定
volatile uint16_t DL_logged_data_array[16];
volatile uint8_t Erase_flag = 0;
volatile uint16_t Flash_value = 0;
volatile uint32_t log_addr = 0, DL_times = 0, Log_times_count = 0;
volatile uint8_t File_Inf0[20];//DATA LOG的檔頭資訊陣列 .
volatile uint8_t File_Inf1[20];//DATA LOG的檔頭資訊陣列 .
volatile uint8_t Download_Flag = 0;
volatile uint8_t DataLogEnd_Flag = 0;
volatile uint8_t Log_SET = 0; //Log_SET 用來記錄目前DATA LOG使用那個SET.

void Data_Log_File0(void);
void Data_Log_File1(void);
volatile uint8_t File_Inf0_Flag = 1, File_Inf1_Flag = 1;
uint16_t Address_Calculate(uint32_t);
void Log_Data_Address_Calculate(void);
volatile uint16_t DL_Calculate = 0;
volatile uint8_t DL_Calculate_Exit = 0;
volatile uint32_t Data_count = 0;
volatile uint8_t StopDataLog_Flag = 0;



void ReDevice_Init(void);

void RestorePinSetting(void);
void SavePinSetting(void);
volatile uint32_t _OUT_Setting[6];
volatile uint32_t _Pin_Setting[11];
volatile uint32_t _PullUp_Setting[6];
volatile uint32_t _PMD_Setting[6];
volatile uint32_t _OFFD_Setting[6];


volatile uint32_t 	flash_addr = 0;
volatile int8_t led_count;
volatile int8_t led_count1;
volatile uint8_t led_mode;
volatile uint8_t key_off = 0;

int32_t Dew_Point(int32_t, int16_t);
int32_t Web_Bulb(int32_t);  //void Dew2T(void)
//volatile int32_t Dew_T1 = 0;

void write_fmc1(uint8_t x, uint8_t y, uint32_t addr);  	//x=1 y=1 write s/n cali date,x=0 write offset(y==0) or cali value(y==1)
void read_fmc1(uint8_t x, uint8_t y, uint32_t addr);		//x=1 y=1 read s/n cali date,x=0 read offset(y==0) or cali value(y==1)

volatile uint8_t ad_cont_set = 8;
//--------------------------------------------------------------------------------------------------------------------------
//------------------battery percentage-----------------
#define Batt_Magnification 0.6818181818   // 300/440

#define Batt_Life_Min 15660   // 261(H)*60(M)
float battV = 0; //battery percentage use
float RBattPercent = 0; //battery percentage use
void Batt_Life(void);
volatile uint16_t BattLifeM = 0;
volatile uint8_t BattLifeLow_Flag = 0;	//當前電池可用時間低於LOG持續時間
float Batt_Percentage(float);  //
volatile uint8_t BattPercent = 0;
volatile float   Batt_Voltage = 0;
//------------------------------------------------------

#endif


#ifdef __cplusplus
}
#endif


/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/


