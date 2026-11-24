enum conneco_cmd {
	CMD_RW_MODULE_INFO = 0,
	CMD_START_PERIPHERAL,
	CMD_HOST_SYNC,
	CMD_OTA_REQ_P_CODE,
	CMD_OTA_SYNC_PACKNO,
	CMD_OTA_UPD_APROM,
	CMD_LOG_SETUP,
	CMD_DOWNLOAD_LOG,
	CMD_WRITE_SN,
	CMD_TEST_RSSI
};

#pragma pack(push) /* push current alignment to stack */
#pragma pack(1) /* set alignment to 1 byte boundary */

struct meas_common {
	uint8_t rp_level : 4; /* reserved */
	uint8_t apo : 4;
	uint16_t ver;
	uint8_t time[3];
};

struct rh_meas_pload {
	int16_t db;
	int16_t wb;
	int16_t humidity;
	int16_t dp;
	int16_t enthalpy;
	struct meas_common common;
};

struct pc_meas_pload {
	int16_t temp;
	struct meas_common common;
};

struct pr_meas_pload {
	int16_t sensor;
	uint16_t baro;
	int16_t psia;
	int16_t psig;
	struct meas_common common;
};

struct vac_meas_pload {
	uint8_t micron[3];
	int16_t leak_rate;
	struct meas_common common;
};

struct mn_meas_pload {
	int16_t pres;
	struct meas_common common;
};

struct cmd_u_tx {
	uint8_t header;
	uint8_t len;
	uint8_t cmd;
	uint8_t ad_len;
	uint8_t ad_type;
	uint8_t fp_code[2];
	uint8_t p_code[2];
	uint8_t sn[4];
	uint8_t batt_level : 4;
	uint8_t r_b : 4;
	uint8_t cal_date[4];
	union {
		struct rh_meas_pload rh;
		struct pc_meas_pload pc;
		struct pr_meas_pload pr;
		struct vac_meas_pload vac;
		struct mn_meas_pload mn;
		uint8_t pload[16];
	} meas;
	uint8_t hw_ver[2];
	struct host_event host;
	struct button_event button;
	uint8_t cs;
	uint8_t end[2];
};

struct cmd_u_rx {
	uint8_t header;
	uint8_t len;
	uint8_t cmd;
	uint8_t fp_gatt;
	uint8_t set_cnt;
	struct user_setting set;
	uint8_t cs;
	uint8_t end[2];
};

struct cmd_q_rx {
	uint8_t header;
	uint8_t len;
	uint8_t cmd;
	struct ble_module_info info;
	uint8_t cs;
	uint8_t end[2];
};

#pragma pack(pop) /* restore original alignment from stack */
