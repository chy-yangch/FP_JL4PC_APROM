#include <stdio.h>
#include <string.h>
#include "conneco_ble.h"
#include "conneco_protocol.h"

#define GPIO_IN(x) (x.port->PMD &= ~(BIT0 << (x.pin * 2)))
#define GPIO_OUT(x) (x.port->PMD |= (BIT0 << (x.pin * 2)))

#define UART_RX_SIZE 64
#define UART_TX_SIZE 64

enum ble_ota_state {
	BLE_OTA_WAIT_START = 0,
	BLE_OTA_TRANSFER,
	BLE_OTA_WAIT_TX_DONE
};

struct ble_ota_context {
	enum ble_ota_state state;
	uint32_t remaining;
	uint32_t expected_packno;
};

struct ble_product_ops {
	uint8_t meas_len;
	uint8_t offset;
	void (*pack_meas)(uint8_t *dst, const struct ble_dynamic_data *data);
};

struct ble_tx_power {
	uint8_t write;
	uint8_t setting;
};

struct ble_internal_context {
	struct ble_context *ctx;
	uint8_t ticks_100ms;
	uint8_t received;
	uint8_t tx_done;
	struct ble_ota_context ota;
	struct ble_product_ops ops;
	struct ble_tx_power power;
};

static struct ble_internal_context ble = {
	.ctx = NULL,
	.ticks_100ms = 0,
	.received = 0,
	.tx_done = 0,
	.ota = {
		.state = BLE_OTA_WAIT_START,
		.remaining = 0,
		.expected_packno = 0
	},
	.power = {
		.write = 0,
		.setting = 0
	}
};

static const uint8_t CMD_SYNC_PACKNO = 0xA4;
static const uint8_t CMD_UPDATE_APROM = 0xA0;

static UART_T *uart = UART0;

static uint8_t rx_data[UART_RX_SIZE] = {0};
static uint8_t tx_data[UART_TX_SIZE] = {0};
static uint8_t ble_mode = 0;
static uint8_t rx_order = 0;
static uint8_t rx_cnt = 0;
static uint8_t recv_size = 0;
static uint8_t prev_cnt = 0;

static void ble_uart_init(void)
{
	IRQn_Type irq = (ble.ctx->hw.uart_port) ? UART1_IRQn : UART0_IRQn;

	/* 115200, 8, N, 1 */
	uart->BAUD = UART_BAUD_MODE0 |
		     UART_BAUD_MODE0_DIVIDER(12000000, 115200);
	uart->TLCTL = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1 |
		      UART_TLCTL_RFITL_1BYTE | UART_TLCTL_RTS_TRI_LEV_1BYTE;

	uart->IER = UART_IER_RTO_IE_Msk | UART_IER_RDA_IE_Msk;
	NVIC_EnableIRQ(irq);
}

static void ble_pdma_init(void)
{
	uint32_t base = (ble.ctx->hw.uart_port) ? UART1_BASE : UART0_BASE;
	uint32_t tx = (ble.ctx->hw.uart_port) ? PDMA_UART1_TX : PDMA_UART0_TX;

	/* UART Tx PDMA */
	PDMA_Open(1 << ble.ctx->hw.pdma_tx_ch);
	PDMA_SetTransferCnt(ble.ctx->hw.pdma_tx_ch, PDMA_WIDTH_8, UART_TX_SIZE);
	PDMA_SetTransferAddr(ble.ctx->hw.pdma_tx_ch, (uint32_t)tx_data,
			     PDMA_SAR_INC, base, PDMA_DAR_FIX);
	PDMA_SetTransferMode(ble.ctx->hw.pdma_tx_ch, tx, 0, 0);
	PDMA_SetTimeOut(ble.ctx->hw.pdma_tx_ch, 0, 0x5555);
	PDMA_EnableInt(ble.ctx->hw.pdma_tx_ch, PDMA_IER_TD_IE_Msk);
	uart->CTL |= UART_CTL_DMA_TX_EN_Msk;

	NVIC_EnableIRQ(PDMA_IRQn);
}

static void gpio_write(const struct gpio *gpio, uint8_t val)
{
	if (val)
		gpio->port->DOUT |= (BIT0 << gpio->pin);
	else
		gpio->port->DOUT &= ~(BIT0 << gpio->pin);
}

static uint8_t gpio_read(const struct gpio *gpio)
{
	return ((gpio->port->PIN & (BIT0 << gpio->pin)) ? 1 : 0);
}

static void ble_wakeup(void)
{
	gpio_write(&ble.ctx->hw.wakeup, 0);
	CLK_SysTickDelay(7000); /* 7 ms */
}

static void ble_sleep(void)
{
	CLK_SysTickDelay(5); /* 5 us */
	gpio_write(&ble.ctx->hw.wakeup, 1);
}

static uint8_t calc_cs_u8(uint8_t *data, uint8_t len)
{
	uint32_t res = 0;

	while (len--)
		res += *data++;

	return ((uint8_t)res);
}

static uint16_t calc_cs_u16(uint8_t *data, uint8_t len)
{
	uint32_t res = 0;

	while (len--)
		res += *data++;

	return ((uint16_t)res);
}

static uint32_t calc_cs_u32(uint8_t *data, uint8_t len)
{
	uint32_t res = 0;

	while (len--)
		res += *data++;

	return res;
}

static void byte_swap(uint8_t *dst, uint8_t *src, uint8_t size)
{
	while (size--)
		*dst++ = *(src + size);
}

static void ble_send_packet(enum conneco_cmd cmd)
{
	uint8_t tx_len = 0;
	uint32_t packno = 0;
	uint32_t cs = 0;

	memset(tx_data, 0, UART_TX_SIZE);
	switch (cmd) {
	case CMD_RW_MODULE_INFO:
		tx_len = 8;
		tx_data[2] = 'Q';
		tx_data[3] = ble.power.write;
		tx_data[4] = ble.power.setting;
		break;
	case CMD_START_PERIPHERAL:
		tx_len = 14;
		tx_data[2] = 'M';
		memcpy(&tx_data[3], ble.ctx->product.sn, 4);
		memcpy(&tx_data[7], ble.ctx->product.hw_ver, 2);
		memcpy(&tx_data[9], ble.ctx->product.p_code, 2);
		break;
	case CMD_HOST_SYNC: {
		/* 組成 cmd U 封包 */
		struct cmd_u_tx *tx = (struct cmd_u_tx *)&tx_data;

		tx_len = 49;
		tx->cmd = 'U';

		struct ble_dynamic_data data;

		/* 取得動態資料 */
		ble.ctx->cb.get_dynamic_data(&data);

		/* 14 = size of ad_type ~ cal_date */
		tx->ad_len = 14 + ble.ops.meas_len;
		tx->ad_type = 0xFF;
		memcpy(tx->fp_code, "FP", 2);
		memcpy(tx->p_code, ble.ctx->product.p_code, 2);
		memcpy(tx->sn, ble.ctx->product.sn, 4);
		tx->batt_level = data.batt_level;
		tx->r_b = data.r_b;
		memcpy(tx->cal_date, ble.ctx->product.cal_date, 4);

		/* 量測值組成封包 */
		ble.ops.pack_meas(tx->meas.pload, &data);

		/* 量測值共同項組成封包 */
		struct meas_common *common =
			(struct meas_common *)&tx->meas.pload[ble.ops.offset];

		common->rp_level = data.rp_level;
		common->apo = data.apo;
		common->ver = ble.ctx->product.fw_ver;
		memcpy(common->time, data.time, 3);

		memcpy(tx->hw_ver, ble.ctx->product.hw_ver, 2);
		tx->host = data.host;
		tx->button = data.button;
		break;
	}
	case CMD_OTA_REQ_P_CODE:
		tx_len = 10;
		tx_data[2] = 'E';
		memcpy(&tx_data[3], ble.ctx->product.hw_ver, 2);
		memcpy(&tx_data[5], ble.ctx->product.p_code, 2);
		break;
	case CMD_OTA_SYNC_PACKNO:
		tx_data[0] = 0xA6;
		packno = *((uint32_t *)&rx_data[4]) + 1;
		*((uint32_t *)&tx_data[4]) = packno;
		tx_len = 16;
		break;
	case CMD_OTA_UPD_APROM:
		cs = calc_cs_u32(rx_data, UART_RX_SIZE);
		*((uint32_t *)&tx_data[0]) = cs;
		packno = *((uint32_t *)&rx_data[4]) + 1;
		*((uint32_t *)&tx_data[4]) = packno;
		tx_len = 16;
		break;
	case CMD_WRITE_SN: {
		tx_len = 18;
		tx_data[2] = rx_data[2];

		struct product_info info = {0};
		enum ble_sn_status st = BLE_WRITE_SN;

		/* 
		 * 因為 struct product_info 的 fw_ver 是 uint16_t，
		 * 使 compiler 認為是 2-byte alignment，
		 * 用 memcpy 避免未對齊問題
		 */
		switch (rx_data[2]) {
		case 'C':
		case 'F':
			memcpy(&info, &rx_data[3], 12);
			ble.power.write = 1;
			if (rx_data[2] == 'F')
				ble.power.setting = FCC_TX_POWER;
			else
				ble.power.setting = CE_TX_POWER;
			break;
		case 'G':
			st = BEL_CLEAN_SN;
			break;
		case 'H':
			st = BLE_READ_SN;
			break;
		default:
			break;
		}

		ble.ctx->cb.handle_sn(st, &info);
		memcpy(&tx_data[3], &info, 12);
		break;
	}
	case CMD_TEST_RSSI:
		tx_len = 7;
		tx_data[2] = 'T';
		break;
	default:
		break;
	}

	if (cmd != CMD_OTA_SYNC_PACKNO && cmd != CMD_OTA_UPD_APROM) {
		tx_data[0] = '#';
		tx_data[1] = tx_len;
		tx_data[tx_len - 3]= calc_cs_u8(tx_data, tx_len - 3);
		tx_data[tx_len - 2] = 0x0D;
		tx_data[tx_len - 1] = 0x0A;
	}

	PDMA_SetTransferCnt(ble.ctx->hw.pdma_tx_ch, PDMA_WIDTH_8, tx_len);
	PDMA_Trigger(ble.ctx->hw.pdma_tx_ch);
}

static void ble_parse_packet(enum conneco_cmd cmd)
{
	uint16_t cs = 0;
	uint32_t addr = 0;

	switch (cmd) {
	case CMD_RW_MODULE_INFO: {
		struct cmd_q_rx *rx = (struct cmd_q_rx *)&rx_data;

		if (rx->cmd != 'Q' ||
		    rx->cs != calc_cs_u8(rx_data, rx->len - 3))
			return;

		ble.ctx->info = rx->info;
		ble.ctx->state = BLE_STATE_PERIOD_SYNC;
		ble.power.write = 0;
		break;
	}
	case CMD_HOST_SYNC: {
		struct cmd_u_rx *rx = (struct cmd_u_rx *)&rx_data;

		if (rx->cs != calc_cs_u8(rx_data, rx->len - 3))
			return;

		if (rx->fp_gatt && prev_cnt != rx->set_cnt) {
			ble.ctx->cb.setting_changed(&rx->set);
			prev_cnt = rx->set_cnt;
		}
		break;
	}
	case CMD_OTA_SYNC_PACKNO:
		ble.ota.expected_packno = 3;
		ble_send_packet(CMD_OTA_SYNC_PACKNO);
		break;
	case CMD_OTA_UPD_APROM: {
		uint32_t cur_packno = *((uint32_t *)&rx_data[4]);
		if (cur_packno != ble.ota.expected_packno) {
			ble_send_packet(CMD_OTA_UPD_APROM);
			return;
		}

		struct ble_ota_params params;

		if (rx_data[0] == CMD_UPDATE_APROM) {
			params.event = BLE_OTA_STARTED;
			params.data = NULL;
			params.len = 0;
			ble.ctx->cb.app_ota_event(&params);

			params.event = BLE_OTA_DATA;
			params.data = &rx_data[16];
			params.len = 48;
			ble.ctx->cb.app_ota_event(&params);

			ble.ota.remaining = *((uint32_t *)&rx_data[12]) - 48;
		} else {
			params.event = BLE_OTA_DATA;
			params.data = &rx_data[8];
			if (ble.ota.remaining > 56) {
				params.len = 56;
				ble.ota.remaining -= 56;
			} else {
				params.len = ble.ota.remaining;
				ble.ota.remaining = 0;
				ble.ota.state = BLE_OTA_WAIT_TX_DONE;
			}
			ble.ctx->cb.app_ota_event(&params);
		}
		ble.ota.expected_packno = cur_packno + 2;
		ble_send_packet(CMD_OTA_UPD_APROM);
		break;
	}
	case CMD_LOG_SETUP:
		byte_swap((uint8_t *)&cs, &rx_data[18], 2);
		if (cs != calc_cs_u16(rx_data, 18))
			return;

		/* tx_data[18] = batt_percent */
		ble.ctx->cb.set_log((struct log_setup *)&rx_data[1],
				    &tx_data[18]);

		memcpy(tx_data, rx_data, 18);
		tx_data[19] = calc_cs_u8(tx_data, 19);
		PDMA_SetTransferCnt(ble.ctx->hw.pdma_tx_ch, PDMA_WIDTH_8, 20);
		PDMA_Trigger(ble.ctx->hw.pdma_tx_ch);
		break;
	case CMD_DOWNLOAD_LOG:
		byte_swap((uint8_t *)&cs, &rx_data[6], 2);
		if (cs != calc_cs_u16(rx_data, 6))
			return;

		byte_swap((uint8_t *)&addr, &rx_data[2], 3);
		if (addr < 2) {
			tx_data[0] = 0;
			tx_data[1] = (uint8_t)addr;
			ble.ctx->cb.get_log_data(addr, &tx_data[2]);
			cs = calc_cs_u16(&tx_data[2], 16); /* cs w/o addr */
			byte_swap(&tx_data[18], (uint8_t *)&cs, 2);
		} else {
			memcpy(&tx_data[0], &rx_data[2], 3);
			memset(&tx_data[3], 0, 16); /* clear payload */
			ble.ctx->cb.get_log_data(addr, &tx_data[3]);
			tx_data[19] = calc_cs_u8(tx_data, 19); /* cs w/ addr */
		}
		PDMA_SetTransferCnt(ble.ctx->hw.pdma_tx_ch, PDMA_WIDTH_8, 20);
		PDMA_Trigger(ble.ctx->hw.pdma_tx_ch);
		break;
	default:
		break;
	}
}

/* 組成 RH 量測封包 */
static void pack_rh_meas(uint8_t *dst, const struct ble_dynamic_data *data)
{
	struct rh_meas_pload *rh = (struct rh_meas_pload *)dst;

	rh->db = data->meas.rh.db;
	rh->wb = data->meas.rh.wb;
	rh->humidity = data->meas.rh.humidity;
	rh->dp = data->meas.rh.dp;
	rh->enthalpy = data->meas.rh.enthalpy;
}

/* 組成 PC 量測封包 */
static void pack_pc_meas(uint8_t *dst, const struct ble_dynamic_data *data)
{
	struct pc_meas_pload *pc = (struct pc_meas_pload *)dst;

	pc->temp = data->meas.pc.temp;
}

/* 組成 PR 量測封包 */
static void pack_pr_meas(uint8_t *dst, const struct ble_dynamic_data *data)
{
	struct pr_meas_pload *pr = (struct pr_meas_pload *)dst;

	pr->sensor = data->meas.pr.sensor;
	pr->baro = data->meas.pr.baro;
	pr->psia = data->meas.pr.psia;
	pr->psig = data->meas.pr.psig;
}

/* 組成 VAC 量測封包 */
static void pack_vac_meas(uint8_t *dst, const struct ble_dynamic_data *data)
{
	struct vac_meas_pload *vac = (struct vac_meas_pload *)dst;

	memcpy(vac->micron, (uint8_t *)&data->meas.vac.micron, 3);
	vac->leak_rate = data->meas.vac.leak_rate;
}

/* 組成 MN 量測封包 */
static void pack_mn_meas(uint8_t *dst, const struct ble_dynamic_data *data)
{
	struct mn_meas_pload *mn = (struct mn_meas_pload *)dst;

	mn->pres = data->meas.mn.pres;
}

/* 找出產品組成封包格式 */
static void get_product_ops(const uint8_t p_code[2])
{
	if (!memcmp(p_code, "BH", 2)) {
		ble.ops.meas_len = sizeof(struct rh_meas_pload);
		ble.ops.pack_meas = pack_rh_meas;
	} else if (!memcmp(p_code, "BF", 2)) {
		ble.ops.meas_len = sizeof(struct pc_meas_pload);
		ble.ops.pack_meas = pack_pc_meas;
	} else if (!memcmp(p_code, "BG", 2)) {
		ble.ops.meas_len = sizeof(struct pr_meas_pload);
		ble.ops.pack_meas = pack_pr_meas;
	} else if (!memcmp(p_code, "BV", 2)) {
		ble.ops.meas_len = sizeof(struct vac_meas_pload);
		ble.ops.pack_meas = pack_vac_meas;
	} else { /* MN */
		ble.ops.meas_len = sizeof(struct mn_meas_pload);
		ble.ops.pack_meas = pack_mn_meas;
	}
	ble.ops.offset = ble.ops.meas_len - sizeof(struct meas_common);
}

void ble_power_on(void)
{
	if (ble.ctx->state == BLE_STATE_POWER_OFF)
		ble.ctx->state = BLE_STATE_POWER_ON;
}

void ble_power_off(void)
{
	if (ble.ctx->state != BLE_STATE_POWER_OFF) {
		gpio_write(&ble.ctx->hw.en, 1);
		gpio_write(&ble.ctx->hw.wakeup, 0);
		GPIO_IN(ble.ctx->hw.rx);
		GPIO_IN(ble.ctx->hw.tx);
		ble.ctx->cb.enable_uart_pin(0);
		ble.ctx->state = BLE_STATE_POWER_OFF;
		ble.ota.state = BLE_OTA_WAIT_START;
		ble_mode = 0;
		prev_cnt = 0;
	}
}

void ble_init(const struct ble_context *ctx)
{
	ble.ctx = (struct ble_context *)ctx;

	GPIO_OUT(ble.ctx->hw.en);
	GPIO_OUT(ble.ctx->hw.wakeup);
	GPIO_OUT(ble.ctx->hw.n_rst);
	GPIO_IN(ble.ctx->hw.chy_gatt_active);
	GPIO_IN(ble.ctx->hw.ota_active);

	gpio_write(&ble.ctx->hw.n_rst, 1);

	uart = (ble.ctx->hw.uart_port) ? UART1 : UART0;

	ble_uart_init();
	ble_pdma_init();

	get_product_ops(ble.ctx->product.p_code);
}

void ble_process(void)
{
	if (ble.tx_done) {
		if (ble.ota.state == BLE_OTA_WAIT_TX_DONE) {
			struct ble_ota_params params = {
				.event = BLE_OTA_FINISHED,
				.data = NULL,
				.len = 0
			};
			ble.ctx->cb.app_ota_event(&params);
			ble.ota.state = BLE_OTA_WAIT_START;
		}
		ble.tx_done = 0;
	} else {
		switch (ble.ctx->state) {
		case BLE_STATE_POWER_OFF:
			break;
		case BLE_STATE_POWER_ON:
			gpio_write(&ble.ctx->hw.en, 0);
			CLK_SysTickDelay(100000); /* 100 ms */
			gpio_write(&ble.ctx->hw.wakeup, 1);
			ble.ctx->cb.enable_uart_pin(1);
			ble.ctx->state = BLE_STATE_INFO;
			break;
		case BLE_STATE_INFO:
			if (ble.ticks_100ms >= 5) {
				ble_wakeup();
				ble_send_packet(CMD_RW_MODULE_INFO);
				ble_sleep();
				ble.ticks_100ms = 0;
			}
			break;
		case BLE_STATE_PERIOD_SYNC:
			if (gpio_read(&ble.ctx->hw.chy_gatt_active)) {
				ble.ctx->state = BLE_STATE_CONNECTED;
			} else if (gpio_read(&ble.ctx->hw.ota_active)) {
				ble.ctx->state = BLE_STATE_MODULE_OTA;
			} else if (ble.ticks_100ms >= 10) {
				ble_wakeup();
				ble_send_packet(CMD_HOST_SYNC);
				ble_sleep();
				ble.ticks_100ms = 0;
			}
			break;
		case BLE_STATE_CONNECTED:
			if (!gpio_read(&ble.ctx->hw.chy_gatt_active)) {
				ble.ctx->state = (ble.power.write) ?
						 BLE_STATE_INFO : 
						 BLE_STATE_PERIOD_SYNC;
				ble.ota.state = BLE_OTA_WAIT_START;
			}
			break;
		case BLE_STATE_MODULE_OTA:
			if (!gpio_read(&ble.ctx->hw.ota_active)) {
				ble.ctx->state = BLE_STATE_INFO;
				ble.ota.state = BLE_OTA_WAIT_START;
			}
			break;
		default:
			break;
		}
	}

	if (!ble.received)
		return;

	ble.received = 0;

	switch (ble.ctx->state) {
	case BLE_STATE_POWER_OFF:
		break;
	case BLE_STATE_POWER_ON:
		break;
	case BLE_STATE_INFO:
		ble_parse_packet(CMD_RW_MODULE_INFO);
		break;
	case BLE_STATE_PERIOD_SYNC:
		if (rx_data[2] == 'M')
			ble_mode = 1;
		else if (rx_data[2] == 'U')
			ble_parse_packet(CMD_HOST_SYNC);
		break;
	case BLE_STATE_CONNECTED:
		if (rx_data[0] == '&') {
			ble_parse_packet(CMD_LOG_SETUP);
		} else if (rx_data[0] == '$') {
			ble_parse_packet(CMD_DOWNLOAD_LOG);
		} else if (rx_data[0] == '>') {
			if (rx_data[rx_data[1] - 3] !=
			    calc_cs_u8(rx_data, rx_data[1] - 3))
				break;

			switch (rx_data[2]) {
			case 'E':
				ble_send_packet(CMD_OTA_REQ_P_CODE);
				ble.ota.state = BLE_OTA_TRANSFER;
				break;
			case 'C':
			case 'F':
			case 'G':
			case 'H':
				ble_send_packet(CMD_WRITE_SN);
				break;
			case 'T':
				ble_send_packet(CMD_TEST_RSSI);
				break;
			}
		} else if (ble.ota.state == BLE_OTA_TRANSFER) {
			switch (rx_data[0]) {
			case CMD_SYNC_PACKNO:
				ble_parse_packet(CMD_OTA_SYNC_PACKNO);
				break;
			case CMD_UPDATE_APROM:
			case 0x00:
				ble_parse_packet(CMD_OTA_UPD_APROM);
				break;
			default:
				break;
			}
		}
		break;
	default:
		break;
	}
}

void ble_tick_100ms(void)
{
	switch (ble.ctx->state) {
	case BLE_STATE_INFO:
	case BLE_STATE_PERIOD_SYNC:
		if (!ble_mode)
			ble.ticks_100ms++;
		else
			ble.ticks_100ms = 0;
		break;
	default:
		ble.ticks_100ms = 0;
		break;
	}
}

void ble_pdma_handler(uint32_t status)
{
	if (status & (BIT0 << ble.ctx->hw.pdma_tx_ch)) {
		ble.tx_done = 1;
		PDMA_CLR_CH_INT_FLAG(ble.ctx->hw.pdma_tx_ch,
				     PDMA_ISR_TD_IS_Msk);
	}
}

void ble_uart_rx_handler(void)
{
	uint8_t dat = 0;

	if (uart->ISR & (UART_ISR_RTO_IS_Msk | UART_ISR_RDA_IS_Msk)) {
		while (uart->FSR & UART_FSR_RX_EMPTY_F_Msk);
		dat = uart->RBR;
		switch (rx_order) {
		case 0:
			if (dat == '>' || dat == '&' || dat == '$' ||
			    ble.ota.state >= BLE_OTA_TRANSFER) {
				rx_data[0] = dat;
				rx_cnt = 1;
				if (dat == '>') {
					rx_order = 1;
					recv_size = 0;
				} else {
					rx_order = 2;
					if (dat == '&')
						recv_size = 20;
					else if (dat == '$')
						recv_size = 8;
					else
						recv_size = UART_RX_SIZE;
				}
			} else {
				rx_cnt = 0;
				recv_size = 0;
			}
			break;
		case 1:
			if (rx_cnt < UART_RX_SIZE) {
				rx_data[rx_cnt++] = dat;
			} else {
				rx_cnt = 0;
				rx_order = 0;
				break;
			}
			if (rx_cnt >= 2 &&
			    rx_data[rx_cnt - 2] == 0x0D &&
			    rx_data[rx_cnt - 1] == 0x0A) {
				rx_cnt = 0;
				rx_order = 0;
				ble.received = 1;
			}
			break;
		case 2:
			rx_data[rx_cnt++] = dat;
			if (rx_cnt >= recv_size) {
				rx_cnt = 0;
				rx_order = 0;
				ble.received = 1;
			}
			break;
		default:
			rx_cnt = 0;
			rx_order = 0;
			recv_size = 0;
			break;
		}
	}
}
