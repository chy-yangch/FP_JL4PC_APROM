#include "Nano1X2Series.h"

#define OTA_BIN_START_ADDR		0x3F8000

#define SPI_FLASH_PORT  SPI1
#define SPI_WRITE_TX    SPI_WRITE_TX1
#define SPI_READ_RX     SPI_READ_RX1

enum flash_error {
	FLASH_ERR_NONE = 0,
	FLASH_ERR_SPI,
	FLASH_ERR_DEVICE
};

void SpiFlash_ChipErase_logger(uint8_t sets);
void SpiFlash_NormalPageProgram(uint32_t Address, uint16_t DataBuffer);
//void SpiFlash_NormalPageProgram(uint32_t Address, uint16_t DataBuffer);
void SpiFlash_WaitReady(void);
int16_t SpiFlash_NormalRead(uint32_t Address);

static const uint16_t EXPECTED_ID = 0xEF15;

void SpiFlash_ChipErase_logger(uint8_t sets)
{

//  0 sets:  0x000000 ~ 0x1FFFFF
//  1 sets:  0x200000 ~ 0x3FFFFF

//set 0:  0x000000  ~  0x0FFFFF
//set 1:  0x100000  ~  0x1FFFFF
//set 2:  0x200000  ~  0x2FFFFF
//set 3:  0x300000  ~  0x3FFFFF
// 資料分別 4個 sets

	uint32_t addre;
	uint32_t addre1;
	uint8_t i;

	sets = sets * 16;
	for (i = 0; i < 16; i++) {
		addre1 = sets + i;
		addre = addre1 << 16;
		SPI_SET_SS0_LOW(SPI_FLASH_PORT);       // /CS: active
		SPI_WRITE_TX(SPI_FLASH_PORT, 0x06);    // send Command: 0x06, Write enable
		while (SPI_IS_BUSY(SPI_FLASH_PORT));   // wait tx finish
		SPI_SET_SS0_HIGH(SPI_FLASH_PORT);      // /CS: de-active
		SPI_SET_SS0_LOW(SPI_FLASH_PORT);        // /CS: active
		SPI_WRITE_TX(SPI_FLASH_PORT, 0xD8);     // send Command: 0xC7, Chip Erase

		SPI_WRITE_TX(SPI_FLASH_PORT, (addre >> 16) & 0xFF);
		SPI_WRITE_TX(SPI_FLASH_PORT, (addre >> 8)  & 0xFF);
		SPI_WRITE_TX(SPI_FLASH_PORT, addre       & 0xFF);

		while (SPI_IS_BUSY(SPI_FLASH_PORT));    // wait tx finish
		SPI_SET_SS0_HIGH(SPI_FLASH_PORT);       //CS: de-active
		SPI_ClearRxFIFO(SPI1);
		SpiFlash_WaitReady();
	}

}


enum flash_error SpiFlash_ReadMidDid(void)
{
	uint8_t u8RxData[6], u8IDCnt = 0;
	uint32_t timeout = 500UL;

	// /CS: active
	SPI_SET_SS0_LOW(SPI_FLASH_PORT);

	// send Command: 0x90, Read Manufacturer/Device ID
	SPI_WRITE_TX(SPI_FLASH_PORT, 0x90);

	// send 24-bit '0', dummy
	SPI_WRITE_TX(SPI_FLASH_PORT, 0x00);
	SPI_WRITE_TX(SPI_FLASH_PORT, 0x00);
	SPI_WRITE_TX(SPI_FLASH_PORT, 0x00);

	// receive 16-bit
	SPI_WRITE_TX(SPI_FLASH_PORT, 0x00);
	SPI_WRITE_TX(SPI_FLASH_PORT, 0x00);

	// wait tx finish
	while (SPI_IS_BUSY(SPI_FLASH_PORT)) {
		timeout--;
		if (timeout == 0)
			return FLASH_ERR_SPI;
	}

	// /CS: de-active
	SPI_SET_SS0_HIGH(SPI_FLASH_PORT);

	while (!SPI_GET_RX_FIFO_EMPTY_FLAG(SPI_FLASH_PORT))
		u8RxData[u8IDCnt ++] = SPI_READ_RX(SPI_FLASH_PORT);

	uint16_t id = (u8RxData[4] << 8) | u8RxData[5];

	if (id != EXPECTED_ID)
		return FLASH_ERR_DEVICE;
	else
		return FLASH_ERR_NONE;
}

void SpiFlash_ChipErase(void)
{
	// /CS: active
	SPI_SET_SS0_LOW(SPI_FLASH_PORT);

	// send Command: 0x06, Write enable
	SPI_WRITE_TX(SPI_FLASH_PORT, 0x06);

	// wait tx finish
	while (SPI_IS_BUSY(SPI_FLASH_PORT));

	// /CS: de-active
	SPI_SET_SS0_HIGH(SPI_FLASH_PORT);

	// /CS: active
	SPI_SET_SS0_LOW(SPI_FLASH_PORT);

	// send Command: 0xC7, Chip Erase
	SPI_WRITE_TX(SPI_FLASH_PORT, 0xC7);

	// wait tx finish
	while (SPI_IS_BUSY(SPI_FLASH_PORT));

	// /CS: de-active
	SPI_SET_SS0_HIGH(SPI_FLASH_PORT);

	SPI_ClearRxFIFO(SPI1);
	SpiFlash_WaitReady();
}

uint8_t SpiFlash_ReadStatusReg(void)
{
	// /CS: active
	SPI_SET_SS0_LOW(SPI_FLASH_PORT);

	// send Command: 0x05, Read status register
	SPI_WRITE_TX(SPI_FLASH_PORT, 0x05);

	// read status
	SPI_WRITE_TX(SPI_FLASH_PORT, 0x00);

	// wait tx finish
	while (SPI_IS_BUSY(SPI_FLASH_PORT));

	// /CS: de-active
	SPI_SET_SS0_HIGH(SPI_FLASH_PORT);

	// skip first rx data
	SPI_READ_RX(SPI_FLASH_PORT);

	return (SPI_READ_RX(SPI_FLASH_PORT) & 0xff);
}



void SpiFlash_WaitReady(void)
{
	uint8_t ReturnValue;

	do {
		ReturnValue = SpiFlash_ReadStatusReg();
		ReturnValue = ReturnValue & 1;
	} while (ReturnValue != 0); // check the BUSY bit
}

//void SpiFlash_NormalPageProgram(uint32_t Address, uint16_t DataBuffer)
void SpiFlash_NormalPageProgram(uint32_t Address, uint16_t DataBuffer)
{
	uint16_t i = 0;

	Address = Address * 2;
	SPI_SET_SS0_LOW(SPI_FLASH_PORT);       // /CS: active
	SPI_WRITE_TX(SPI_FLASH_PORT, 0x06);    // send Command: 0x06, Write enable
	while (SPI_IS_BUSY(SPI_FLASH_PORT));   // wait tx finish
	SPI_SET_SS0_HIGH(SPI_FLASH_PORT);      // /CS: de-active
	SPI_SET_SS0_LOW(SPI_FLASH_PORT);       // /CS: active
	SPI_WRITE_TX(SPI_FLASH_PORT, 0x02);    // send Command: 0x02, Page program
	SPI_WRITE_TX(SPI_FLASH_PORT, (Address >> 16) & 0xFF);  // send 24-bit start address
	SPI_WRITE_TX(SPI_FLASH_PORT, (Address >> 8)  & 0xFF);
	SPI_WRITE_TX(SPI_FLASH_PORT, Address       & 0xFF);

	i = DataBuffer;
	SPI_WRITE_TX(SPI_FLASH_PORT, i);
	i = DataBuffer >> 8;
	SPI_WRITE_TX(SPI_FLASH_PORT, i);

	while (SPI_IS_BUSY(SPI_FLASH_PORT));  // wait tx finish
	SPI_SET_SS0_HIGH(SPI_FLASH_PORT);      // /CS: de-active
	SPI_ClearRxFIFO(SPI_FLASH_PORT);
}

int16_t SpiFlash_NormalRead(uint32_t Address)
{
	uint8_t i;
	uint16_t j;

	Address = Address * 2;
	SPI_SET_SS0_LOW(SPI_FLASH_PORT);   // /CS: active
	SPI_WRITE_TX(SPI_FLASH_PORT, 0x03);  //0x03 send Command: 0x03, Read data
	SPI_WRITE_TX(SPI_FLASH_PORT, (Address >> 16) & 0xFF); // send 24-bit start address
	SPI_WRITE_TX(SPI_FLASH_PORT, (Address >> 8)  & 0xFF);
	SPI_WRITE_TX(SPI_FLASH_PORT, Address       & 0xFF);
	while (SPI_IS_BUSY(SPI_FLASH_PORT));
	SPI_ClearRxFIFO(SPI_FLASH_PORT);    // clear RX buffer


	SPI_WRITE_TX(SPI_FLASH_PORT, 0x00);     // read data
	while (SPI_IS_BUSY(SPI_FLASH_PORT));
	i = SPI_READ_RX(SPI_FLASH_PORT);
	SPI_WRITE_TX(SPI_FLASH_PORT, 0x00);
	while (SPI_IS_BUSY(SPI_FLASH_PORT));
	j = SPI_READ_RX(SPI_FLASH_PORT);
	j = j << 8;
	j |= i;

	while (SPI_IS_BUSY(SPI_FLASH_PORT));    // wait tx finish

	SPI_SET_SS0_HIGH(SPI_FLASH_PORT);       // /CS: de-active

	return j;

}
//----------------------------------------------------
/*
uint16_t SpiFlash_Power_Down(void)
{


  // /CS: active
    SPI_SET_SS0_LOW(SPI_FLASH_PORT);

    // send Command: 0x06, Write enable
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x06);

    // wait tx finish
    while(SPI_IS_BUSY(SPI_FLASH_PORT));

    // /CS: de-active
    SPI_SET_SS0_HIGH(SPI_FLASH_PORT);

    // /CS: active
    SPI_SET_SS0_LOW(SPI_FLASH_PORT);

    // send Command: 0xB9, Power-down
    SPI_WRITE_TX(SPI_FLASH_PORT, 0xB9);


    // wait tx finish
    while(SPI_IS_BUSY(SPI_FLASH_PORT));

    // /CS: de-active
    SPI_SET_SS0_HIGH(SPI_FLASH_PORT);

    SPI_ClearRxFIFO(SPI1);
		SpiFlash_WaitReady();
}
*/

void SpiFlash_ChipErase_4K(uint32_t address)
{
	SpiFlash_WaitReady();
	SPI_SET_SS0_LOW(SPI_FLASH_PORT);       // /CS: active
	SPI_WRITE_TX(SPI_FLASH_PORT, 0x06);    // send Command: 0x06, Write enable
	while (SPI_IS_BUSY(SPI_FLASH_PORT));     // wait tx finish
	SPI_SET_SS0_HIGH(SPI_FLASH_PORT);      // /CS: de-active
	SPI_SET_SS0_LOW(SPI_FLASH_PORT);       // /CS: active
	SPI_WRITE_TX(SPI_FLASH_PORT, 0x20);    // send Command: 0x20, Chip Erase 4K

	SPI_WRITE_TX(SPI_FLASH_PORT, (address >> 16) & 0xFF);
	SPI_WRITE_TX(SPI_FLASH_PORT, (address >> 8)  & 0xFF);
	SPI_WRITE_TX(SPI_FLASH_PORT, address & 0xFF);

	while (SPI_IS_BUSY(SPI_FLASH_PORT));    // wait tx finish
	SPI_SET_SS0_HIGH(SPI_FLASH_PORT);       //CS: de-active
	SPI_ClearRxFIFO(SPI1);
	SpiFlash_WaitReady();
}

void SpiFlash_ChipErase_32K(uint32_t address)
{
	SpiFlash_WaitReady();
	SPI_SET_SS0_LOW(SPI_FLASH_PORT);       // /CS: active
	SPI_WRITE_TX(SPI_FLASH_PORT, 0x06);    // send Command: 0x06, Write enable
	while (SPI_IS_BUSY(SPI_FLASH_PORT));     // wait tx finish
	SPI_SET_SS0_HIGH(SPI_FLASH_PORT);      // /CS: de-active
	SPI_SET_SS0_LOW(SPI_FLASH_PORT);       // /CS: active
	SPI_WRITE_TX(SPI_FLASH_PORT, 0x52);    // send Command: 0x52, Chip Erase 32K

	SPI_WRITE_TX(SPI_FLASH_PORT, (address >> 16) & 0xFF);
	SPI_WRITE_TX(SPI_FLASH_PORT, (address >> 8)  & 0xFF);
	SPI_WRITE_TX(SPI_FLASH_PORT, address & 0xFF);

	while (SPI_IS_BUSY(SPI_FLASH_PORT));    // wait tx finish
	SPI_SET_SS0_HIGH(SPI_FLASH_PORT);       //CS: de-active
	SPI_ClearRxFIFO(SPI1);
	SpiFlash_WaitReady();
}

void SpiFlash_ChipErase_64K(uint32_t address)
{
	SpiFlash_WaitReady();
	SPI_SET_SS0_LOW(SPI_FLASH_PORT);       // /CS: active
	SPI_WRITE_TX(SPI_FLASH_PORT, 0x06);    // send Command: 0x06, Write enable
	while (SPI_IS_BUSY(SPI_FLASH_PORT));     // wait tx finish
	SPI_SET_SS0_HIGH(SPI_FLASH_PORT);      // /CS: de-active
	SPI_SET_SS0_LOW(SPI_FLASH_PORT);       // /CS: active
	SPI_WRITE_TX(SPI_FLASH_PORT, 0xD8);    // send Command: 0xD8, Chip Erase 64K

	SPI_WRITE_TX(SPI_FLASH_PORT, (address >> 16) & 0xFF);
	SPI_WRITE_TX(SPI_FLASH_PORT, (address >> 8)  & 0xFF);
	SPI_WRITE_TX(SPI_FLASH_PORT, address & 0xFF);

	while (SPI_IS_BUSY(SPI_FLASH_PORT));    // wait tx finish
	SPI_SET_SS0_HIGH(SPI_FLASH_PORT);       //CS: de-active
	SPI_ClearRxFIFO(SPI1);
	SpiFlash_WaitReady();
}

void bin_space_erase(void)
{

	//所有Erase的指令皆需對齊邊界位址不可以由任意位址進行清除動作
	//寫入Flash資料位址也需對齊256Bytes邊界位址,不可由任意位址寫入
	//讀取可由任意位址進行讀取
	//X SpiFlash_ChipErase_32K(0x001000);

	SpiFlash_ChipErase_32K(OTA_BIN_START_ADDR);

}

uint8_t W25Q_SPI_ReadByte(void)
{
    uint8_t status;

    SPI_WRITE_TX(SPI_FLASH_PORT, 0x00);     // read data
    while (SPI_IS_BUSY(SPI_FLASH_PORT));
    status  = SPI_READ_RX(SPI_FLASH_PORT);

    return status;
}

uint8_t W25Q_ReadStatus1(void)
{
    uint8_t status;

    SPI_SET_SS0_LOW(SPI_FLASH_PORT);

    SPI_WRITE_TX(SPI_FLASH_PORT,0x05);

    status = W25Q_SPI_ReadByte();

    while (SPI_IS_BUSY(SPI_FLASH_PORT));   	// wait tx finish

    SPI_SET_SS0_HIGH(SPI_FLASH_PORT);      // /CS: de-active

    return status;
}


void W25Q_WaitBusy(void)
{
    while (W25Q_ReadStatus1() & 0x01)
    {
        ;
    }
}

/*
W25Q_Write(uint32_t addr, const uint8_t *buffer, uint32_t len)

PS: W25Q_Write() 只負責：把資料寫到 Flash 已經是 0xFF 的位置,它不負責 Erase。

以 500 Bytes 為例

例如：

W25Q_Write(0x010020, data, 500);

程式會自動拆成：

第一次 Page Program
Address = 0x010020
Length  = 224 Bytes

        ↓

第二次 Page Program
Address = 0x010100
Length  = 256 Bytes

        ↓

第三次 Page Program
Address = 0x010200
Length  = 20 Bytes

因為：

0x010020 ~ 0x0100FF
        = 224 Bytes

0x010100 ~ 0x0101FF
        = 256 Bytes

剩下
500 - 224 - 256
        = 20 Bytes
這樣就完全避開 W25Q32JV 的 Page Wrap 行為。

*/
void W25Q_Write(uint32_t addr, const uint8_t *buffer, uint32_t len)
{
    uint32_t page_remain;
    uint32_t write_len,i;

    if (buffer == NULL || len == 0)
        return;

    while (len > 0)
    {
        /*
         * 目前 Page 剩餘空間
         */
        page_remain = 256UL - (addr & 0xFFUL);

        /*
         * 決定這一次 Page Program 寫多少
         */
        if (len < page_remain)
            write_len = len;
        else
            write_len = page_remain;

        /*
         * 等待 Flash Ready
         */
        //while (SPI_IS_BUSY(SPI_FLASH_PORT));
	W25Q_WaitBusy();

        /*
         * Write Enable
         */
	SPI_SET_SS0_LOW(SPI_FLASH_PORT);       // /CS: active
	SPI_WRITE_TX(SPI_FLASH_PORT, 0x06);    // send Command: 0x06, Write enable
	while (SPI_IS_BUSY(SPI_FLASH_PORT));   	// wait tx finish
	SPI_SET_SS0_HIGH(SPI_FLASH_PORT);      // /CS: de-active

        /*
         * Page Program - 02h
         */
	SPI_SET_SS0_LOW(SPI_FLASH_PORT);       // /CS: active
	SPI_WRITE_TX(SPI_FLASH_PORT, 0x02);    // send Command: 0x02, Page program

        /*
         * 24-bit Address
         */
        SPI_WRITE_TX(SPI_FLASH_PORT,(uint8_t)(addr >> 16));
        SPI_WRITE_TX(SPI_FLASH_PORT,(uint8_t)(addr >> 8));
        SPI_WRITE_TX(SPI_FLASH_PORT,(uint8_t)(addr));

        /*
         * 寫入資料
         */
        for (i = 0; i < write_len; i++)
        {
            SPI_WRITE_TX(SPI_FLASH_PORT,buffer[i]);
        }

	while (SPI_IS_BUSY(SPI_FLASH_PORT));     // wait tx finish
	SPI_SET_SS0_HIGH(SPI_FLASH_PORT);      // /CS: de-active
	SPI_ClearRxFIFO(SPI_FLASH_PORT);

        /*
         * 等待 Flash Ready
         */
	W25Q_WaitBusy();

        /*
         * 移到下一個地址
         */
        addr += write_len;
        buffer += write_len;
        len -= write_len;
    }
}

void W25Q_Read(uint32_t address, uint8_t *buffer, uint32_t len)
{
    uint32_t i;

    if (buffer == NULL || len == 0)
        return;

    SPI_SET_SS0_LOW(SPI_FLASH_PORT);   // /CS: active

    /*
     * Read Data command
     * 03h
     */
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x03);  //0x03 send Command: 0x03, Read data

    /*
     * 24-bit Address
     */
	SPI_WRITE_TX(SPI_FLASH_PORT, (address >> 16) & 0xFF); // send 24-bit start address
	SPI_WRITE_TX(SPI_FLASH_PORT, (address >> 8)  & 0xFF);
	SPI_WRITE_TX(SPI_FLASH_PORT, address       & 0xFF);
	while (SPI_IS_BUSY(SPI_FLASH_PORT));
	SPI_ClearRxFIFO(SPI_FLASH_PORT);    // clear RX buffer

    /*
     * Continuous Read
     */
    for (i = 0; i < len; i++)
    {
	SPI_WRITE_TX(SPI_FLASH_PORT, 0x00);     // read data
	while (SPI_IS_BUSY(SPI_FLASH_PORT));
	buffer[i]  = SPI_READ_RX(SPI_FLASH_PORT);
    }

    while (SPI_IS_BUSY(SPI_FLASH_PORT));    // wait tx finish

    SPI_SET_SS0_HIGH(SPI_FLASH_PORT);       // /CS: de-active

}
