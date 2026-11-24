#include "Nano1X2Series.h"
#ifdef __cplusplus
extern "C"
{
#endif

#define KEY_CLICK_STAR_1_SEC	10
#define KEY_HOLD_TIME	50

struct KEY_STRUCT {

	__IO uint32_t *keyin;
	__IO uint8_t key_lock_bit;
	__IO uint32_t key_hold_cnt;
	__IO uint8_t key_time_start_flag;
	__IO uint8_t key_time_start_cnt;
	__IO uint8_t key_click_cnt;
	__IO uint8_t hold_lock_flag;
	__IO uint8_t key_rtc_time[3];
	__IO enum {KEY_NONE,KEY_CLICK,KEY_PRESS,KEY_DOUBLE_CLICK,KEY_HOLD}key_action_status;

};

extern struct KEY_STRUCT power_key;
extern struct KEY_STRUCT enter_key;
extern struct KEY_STRUCT combo_key;
extern void Key_Scan(void);
