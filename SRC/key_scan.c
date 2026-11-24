#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "key_scan.h"

struct KEY_STRUCT power_key = {

	.keyin =  (uint32_t *)&PA3,
	.key_action_status = KEY_NONE,
	.key_click_cnt = 0,
	.key_hold_cnt = 0,
	.key_lock_bit = 0,
	.key_time_start_cnt = 0,
	.key_time_start_flag = 0,
	.hold_lock_flag = 1,
	.key_rtc_time = {0},
};


struct KEY_STRUCT enter_key = {

	.keyin =  (uint32_t *)&PB15,
	.key_action_status = KEY_NONE,
	.key_click_cnt = 0,
	.key_hold_cnt = 0,
	.key_lock_bit = 0,
	.key_time_start_cnt = 0,
	.key_time_start_flag = 0,
	.key_rtc_time = {0},
};

struct KEY_STRUCT combo_key = {
	.keyin = NULL,
	.key_action_status = KEY_NONE,
	.key_click_cnt = 0,
	.key_hold_cnt = 0,
	.key_lock_bit = 0,
	.key_time_start_cnt = 0,
	.key_time_start_flag = 0,
	.key_rtc_time = {0},
};

void key_status_logic_0(struct KEY_STRUCT *key);
void key_status_logic_1(struct KEY_STRUCT *key);
void combo_key_status_logic(void);

void Key_Scan(void);

void Key_Scan(void)
{
	key_status_logic_1(&enter_key);
	key_status_logic_0(&power_key);
	combo_key_status_logic();
}

void key_status_logic_0(struct KEY_STRUCT *key)
{
    S_RTC_TIME_DATA_T sCurTime;

	if (key->key_time_start_flag)
		key->key_time_start_cnt++;
	else
		__NOP();

	if (!*(key->keyin)) {

		if (key->key_lock_bit == 0) {

			key->key_lock_bit = 1;
			key->key_hold_cnt = 1;

			key->key_time_start_flag = 1;
			key->key_click_cnt++;

		} else {

			if (key->key_hold_cnt  > KEY_HOLD_TIME) {

				key->key_action_status = KEY_HOLD;

				RTC_GetDateAndTime(&sCurTime);
				key->key_rtc_time[0] =  sCurTime.u32Hour;
				key->key_rtc_time[1] =  sCurTime.u32Minute;
				key->key_rtc_time[2] =  sCurTime.u32Second;

			} else {
				key->key_hold_cnt++;
			}
		}
	} else {

		key->key_lock_bit  = 0;

		if (power_key.hold_lock_flag) {

			power_key.hold_lock_flag = 0;
			key->key_hold_cnt = 0;
			key->key_time_start_cnt = 0;
			key->key_time_start_flag = 0;
			key->key_action_status = KEY_NONE;
			key->key_click_cnt = 0;
		}

		if ((key->key_time_start_flag == 0)  && (key->key_action_status == KEY_NONE)){

			key->key_hold_cnt = 0;
			key->key_time_start_cnt = 0;
			key->key_time_start_flag = 0;
			key->key_action_status = KEY_NONE;
			key->key_click_cnt = 0;

		} else {

			if (key->key_time_start_cnt >= KEY_CLICK_STAR_1_SEC) {	// > 1 second

				if (key->key_hold_cnt  < KEY_HOLD_TIME) {

					if (key->key_action_status == KEY_NONE) {

						if (key->key_hold_cnt >= KEY_CLICK_STAR_1_SEC)
							key->key_action_status = KEY_PRESS;
						else
							key->key_action_status = KEY_CLICK;

						RTC_GetDateAndTime(&sCurTime);
						key->key_rtc_time[0] =  sCurTime.u32Hour;
						key->key_rtc_time[1] =  sCurTime.u32Minute;
						key->key_rtc_time[2] =  sCurTime.u32Second;
					}

				} else {
					__NOP();
				}

				key->key_time_start_flag = 0;

			} else {

				if (key->key_click_cnt >= 2) {

					key->key_action_status = KEY_DOUBLE_CLICK;
					key->key_hold_cnt =  key->key_time_start_cnt;
					key->key_click_cnt = 0;
					key->key_time_start_flag = 0;

					RTC_GetDateAndTime(&sCurTime);
					key->key_rtc_time[0] =  sCurTime.u32Hour;
					key->key_rtc_time[1] =  sCurTime.u32Minute;
					key->key_rtc_time[2] =  sCurTime.u32Second;
				}

				__NOP();
			}
		}
	}
}


void key_status_logic_1(struct KEY_STRUCT *key)
{
    S_RTC_TIME_DATA_T sCurTime;

	if (key->key_time_start_flag)
		key->key_time_start_cnt++;
	else
		__NOP();

	if (*(key->keyin)) {

		if (key->key_lock_bit == 0) {

			key->key_lock_bit = 1;
			key->key_hold_cnt = 1;

			key->key_time_start_flag = 1;
			key->key_click_cnt++;

		} else {

			if (key->key_hold_cnt  > KEY_HOLD_TIME) {

				key->key_action_status = KEY_HOLD;

				RTC_GetDateAndTime(&sCurTime);
				key->key_rtc_time[0] =  sCurTime.u32Hour;
				key->key_rtc_time[1] =  sCurTime.u32Minute;
				key->key_rtc_time[2] =  sCurTime.u32Second;

			} else {
				key->key_hold_cnt++;
			}
		}
	} else {

		key->key_lock_bit  = 0;

		if (key->hold_lock_flag) {
			key->key_time_start_flag = 0;
			key->key_action_status = KEY_NONE;
			key->hold_lock_flag = 0;
		}

		if ((key->key_time_start_flag == 0)  && (key->key_action_status == KEY_NONE)){

			key->key_hold_cnt = 0;
			key->key_time_start_cnt = 0;
			key->key_time_start_flag = 0;
			key->key_action_status = KEY_NONE;
			key->key_click_cnt = 0;

		} else {

			if (key->key_time_start_cnt >= KEY_CLICK_STAR_1_SEC) {	// > 1 second

				if (key->key_hold_cnt  < KEY_HOLD_TIME) {


					if (key->key_action_status == KEY_NONE) {

						if (key->key_hold_cnt >= KEY_CLICK_STAR_1_SEC)
							key->key_action_status = KEY_PRESS;
						else
							key->key_action_status = KEY_CLICK;

						RTC_GetDateAndTime(&sCurTime);
						key->key_rtc_time[0] =  sCurTime.u32Hour;
						key->key_rtc_time[1] =  sCurTime.u32Minute;
						key->key_rtc_time[2] =  sCurTime.u32Second;
					}

				} else {
					__NOP();
				}

				key->key_time_start_flag = 0;

			} else {

				if (key->key_click_cnt >= 2) {

					key->key_action_status = KEY_DOUBLE_CLICK;
					key->key_hold_cnt =  key->key_time_start_cnt;
					key->key_click_cnt = 0;
					key->key_time_start_flag = 0;

					RTC_GetDateAndTime(&sCurTime);
					key->key_rtc_time[0] =  sCurTime.u32Hour;
					key->key_rtc_time[1] =  sCurTime.u32Minute;
					key->key_rtc_time[2] =  sCurTime.u32Second;
				}

				__NOP();
			}
		}
	}
}

void combo_key_status_logic(void)
{
	S_RTC_TIME_DATA_T sCurTime;

	if (combo_key.key_time_start_flag)
		combo_key.key_time_start_cnt++;
	else
		__NOP();

	if (!(*power_key.keyin) && *enter_key.keyin) {

		if (combo_key.key_lock_bit == 0) {

			combo_key.key_lock_bit = 1;
			combo_key.key_hold_cnt = 1;

			combo_key.key_time_start_flag = 1;
			combo_key.key_click_cnt++;

		} else {

			if (combo_key.key_hold_cnt  > KEY_HOLD_TIME) {

				combo_key.key_action_status = KEY_HOLD;

				RTC_GetDateAndTime(&sCurTime);
				combo_key.key_rtc_time[0] =  sCurTime.u32Hour;
				combo_key.key_rtc_time[1] =  sCurTime.u32Minute;
				combo_key.key_rtc_time[2] =  sCurTime.u32Second;

			} else {
				combo_key.key_hold_cnt++;
			}
		}
	} else {

		combo_key.key_lock_bit  = 0;

		if (combo_key.hold_lock_flag) {
			combo_key.key_time_start_flag = 0;
			combo_key.key_action_status = KEY_NONE;
			combo_key.hold_lock_flag = 0;
		}

		if ((combo_key.key_time_start_flag == 0)  && (combo_key.key_action_status == KEY_NONE)){

			combo_key.key_hold_cnt = 0;
			combo_key.key_time_start_cnt = 0;
			combo_key.key_time_start_flag = 0;
			combo_key.key_action_status = KEY_NONE;
			combo_key.key_click_cnt = 0;

		} else {

			if (combo_key.key_time_start_cnt >= KEY_CLICK_STAR_1_SEC) {	// > 1 second

				if (combo_key.key_hold_cnt  < KEY_HOLD_TIME) {


					if (combo_key.key_action_status == KEY_NONE) {

						if (combo_key.key_hold_cnt >= KEY_CLICK_STAR_1_SEC)
							combo_key.key_action_status = KEY_PRESS;
						else
							combo_key.key_action_status = KEY_CLICK;

						RTC_GetDateAndTime(&sCurTime);
						combo_key.key_rtc_time[0] =  sCurTime.u32Hour;
						combo_key.key_rtc_time[1] =  sCurTime.u32Minute;
						combo_key.key_rtc_time[2] =  sCurTime.u32Second;
					}

				} else {
					__NOP();
				}

				combo_key.key_time_start_flag = 0;

			} else {

				if (combo_key.key_click_cnt >= 2) {

					combo_key.key_action_status = KEY_DOUBLE_CLICK;
					combo_key.key_hold_cnt =  combo_key.key_time_start_cnt;
					combo_key.key_click_cnt = 0;
					combo_key.key_time_start_flag = 0;

					RTC_GetDateAndTime(&sCurTime);
					combo_key.key_rtc_time[0] =  sCurTime.u32Hour;
					combo_key.key_rtc_time[1] =  sCurTime.u32Minute;
					combo_key.key_rtc_time[2] =  sCurTime.u32Second;
				}

				__NOP();
			}
		}
	}

	if (combo_key.key_action_status > KEY_NONE) {
		power_key.key_action_status = KEY_NONE;
		enter_key.key_action_status = KEY_NONE;
	}
}
