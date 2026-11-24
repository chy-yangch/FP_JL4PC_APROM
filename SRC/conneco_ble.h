#include "Nano1X2Series.h"

#define FCC_TX_POWER 150
#define CE_TX_POWER  80

enum ble_state {
	BLE_STATE_POWER_OFF = 0,
	BLE_STATE_POWER_ON,
	BLE_STATE_INFO,
	BLE_STATE_PERIOD_SYNC,
	BLE_STATE_CONNECTED,
	BLE_STATE_MODULE_OTA,
	BLE_STATE_RESET
};

enum ble_ota_event {
	BLE_OTA_STARTED = 0,
	BLE_OTA_DATA,
	BLE_OTA_FINISHED
};

enum ble_sn_status {
	BLE_WRITE_SN = 0,
	BEL_CLEAN_SN,
	BLE_READ_SN
};

enum host_status {
	HOST_IDLE = 0,
	HOST_READY,
	HOST_DATALOG,
	HOST_ERROR
};

enum button_status {
	BUTTON_NONE = 0,
	BUTTON_CLICK,
	BUTTON_PRESS,
	BUTTON_DOUBLE_CLICK,
	BUTTON_HELD
};

enum button_id {
	BUTTON_ID_POWER = 0,
	BUTTON_ID_MODE,
	BUTTON_ID_ALL
};

struct gpio {
	GPIO_T *port;
	uint8_t pin; /* pin number: 0 ~ 15 */
};

struct product_info { /* 產品資訊 */
	uint8_t sn[4];
	uint8_t hw_ver[2];
	uint8_t p_code[2];
	uint8_t cal_date[4];
	uint16_t fw_ver;
};

struct ble_module_info { /* 模組資訊 */
	uint8_t mac[6];
	uint8_t tx_pwr;
	uint8_t ver;
};

struct ble_ota_params {
	enum ble_ota_event event;
	const uint8_t *data;
	uint16_t len;
};

struct log_setup {
	uint8_t mode; /* = 0: beacon on; = 1: beacon off when logging */
	uint8_t present_time[6];
	uint8_t start_time[6];
	uint8_t interval_sec[2];
	uint8_t duration_min[2];
};

#pragma pack(push) /* push current alignment to stack */
#pragma pack(1) /* set alignment to 1 byte boundary */

struct host_event {
	enum host_status status;
	union {
		struct {
			uint8_t init : 1;
			uint8_t i2c : 1;
			uint8_t spi : 1;
			uint8_t adc : 1;
			uint8_t ext_flash : 1;
			uint8_t cal : 1;
			uint8_t int_flash : 1;
		} bit;
		uint16_t raw;
	} err;
};

struct button_event {
	enum button_status status;
	enum button_id id;
	uint16_t duration_ms;
	uint8_t ts_hour;
	uint8_t ts_min;
	uint8_t ts_sec;
};

#pragma pack(pop) /* restore original alignment from stack */

struct ble_dynamic_data {
	uint8_t batt_level;
	uint8_t r_b;
	union {
		struct {
			int16_t db;
			int16_t wb;
			int16_t humidity;
			int16_t dp;
			int16_t enthalpy;
		} rh;
		struct {
			int16_t temp;
		} pc;
		struct {
			int16_t sensor;
			uint16_t baro;
			int16_t psia;
			int16_t psig;
		} pr;
		struct {
			uint32_t micron;
			int16_t leak_rate;
		} vac;
		struct {
			int16_t pres;
		} mn;
	} meas;
	uint8_t rp_level; /* reserved */
	uint8_t apo;
	uint8_t time[3];
	struct host_event host;
	struct button_event button;
};

struct user_setting {
	uint8_t apo;
	uint8_t r_b;
};

struct ble_callbacks {
	void (*enable_uart_pin)(uint8_t enable);
	void (*get_dynamic_data)(struct ble_dynamic_data *data);
	void (*setting_changed)(const struct user_setting *set);
	void (*app_ota_event)(const struct ble_ota_params *params);
	void (*set_log)(const struct log_setup *setup, uint8_t *batt_percent);
	void (*get_log_data)(uint32_t addr, uint8_t *data);
	void (*handle_sn)(enum ble_sn_status st, struct product_info *info);
};

struct ble_hardware_config { /* 硬體設定 */
	/* UART */
	uint8_t uart_port; /* = 0, 1 */
	uint8_t pdma_rx_ch; /* reserved */
	uint8_t pdma_tx_ch;

	/* GPIO */
	struct gpio rx;
	struct gpio tx;
	struct gpio en;
	struct gpio wakeup;
	struct gpio n_rst;
	struct gpio chy_gatt_active;
	struct gpio ota_active;
};

struct ble_context {
	/* Host -> BLE module */
	struct product_info product;
	struct ble_hardware_config hw;
	struct ble_callbacks cb; /* callback functions */

	/* BLE module -> Host */
	struct ble_module_info info;
	enum ble_state state;
};

void ble_init(const struct ble_context *cxt);
void ble_power_on(void);
void ble_power_off(void);
void ble_process(void);
void ble_tick_100ms(void);
void ble_pdma_handler(uint32_t status);
void ble_uart_rx_handler(void);
