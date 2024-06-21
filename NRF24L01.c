/*
 * NRF24L01.c
 *
 *  Created on: Mar 23, 2024
 *      Author: WELCOME
 */

#include "stm32f4xx_hal.h"
#include "NRF24L01.h"

extern SPI_HandleTypeDef hspi1;
#define NRF24_SPI &hspi1

#define NRF24_CE_PORT GPIOA
#define NRF24_CE_PIN GPIO_PIN_3

#define NRF24_CSN_PORT GPIOA
#define NRF24_CSN_PIN GPIO_PIN_4

void CSN_Select(void)
{
	HAL_GPIO_WritePin(NRF24_CSN_PORT,NRF24_CSN_PIN,GPIO_PIN_RESET);
}
void CSN_UnSelect(void)
{
	HAL_GPIO_WritePin(NRF24_CSN_PORT,NRF24_CSN_PIN,GPIO_PIN_SET);
}

void CE_Enable(void)
{
	HAL_GPIO_WritePin(NRF24_CE_PORT,NRF24_CE_PIN,GPIO_PIN_SET);
}
void CE_Disable(void)
{
	HAL_GPIO_WritePin(NRF24_CE_PORT,NRF24_CE_PIN,GPIO_PIN_RESET);
}





void nrf24_WriteReg(uint8_t Reg, uint8_t Data )
{//used to send single byte so first we'll send buf 0 and then we directly send data next
	uint8_t buf[2];
	buf[0] = Reg|1<<5;
	buf[1] = Data;
	//Pull the CS pin LOW to the device
	CSN_Select();

	HAL_SPI_Transmit(NRF24_SPI,buf,2,1000);

	//Pull the CS High to release the device
	CSN_UnSelect();
}

void nrf24_WriteRegMulti (uint8_t Reg, uint8_t *data, int size)
{//used to send multiple bytes so first we'll send buf 0 and then we directly send data next
	uint8_t buf[2];
		buf[0] = Reg|1<<5;
		//buf[1] = Data;
		//Pull the CS pin LOW to the device
		CSN_Select();

		HAL_SPI_Transmit(NRF24_SPI,buf,2,1000);
		HAL_SPI_Transmit(NRF24_SPI,data,size,1000);

		//Pull the CS High to release the device
		CSN_UnSelect();
}


uint8_t nrf24_ReadReg (uint8_t Reg)
{//used to read single byte from the register
	uint8_t data=0;
	 int size;

	CSN_Select();

	HAL_SPI_Transmit(NRF24_SPI, &Reg, 1, 100);
	HAL_SPI_Receive(NRF24_SPI, data, size, 1000);

	CSN_UnSelect();
	return data;
}

void nrf24_ReadReg_Multi (uint8_t Reg, uint8_t *data, int size)
{//used to read multiple bytes from the register
	CSN_Select();

	HAL_SPI_Transmit(NRF24_SPI, &Reg, 1,100);
	HAL_SPI_Receive(NRF24_SPI, data, size, 1000);

	CSN_UnSelect();

}


void nrfsendcmd (uint8_t cmd)
{
	CSN_Select();

	HAL_SPI_Transmit(NRF24_SPI, &cmd, 1,100);

	CSN_UnSelect();
}


void NRF24_Init (void)
{
	CE_Disable();


	nrf24_WriteReg(CONFIG, 0);// Will be configured later

	nrf24_WriteReg(EN_AA,0);// Disabling Auto acknowledgment

	nrf24_WriteReg(EN_RXADDR, 0);//Will enable later for reception

	nrf24_WriteReg(SETUP_AW, 0x03);// 5 bytes for the TX/RX address

	nrf24_WriteReg(SETUP_RETR,0);//Will do it later-Automatic retransmission

	nrf24_WriteReg (RF_CH,0);//Will do it later - To set the channels

	nrf24_WriteReg (RF_SETUP, 0x0E);//Power=0db, data rate=2mbps

	CE_Enable();

}

void NRF24_TxMode (uint8_t *Address,uint8_t Channel)
{
	CE_Disable();


	nrf24_WriteReg(RF_CH, Channel); //For selecting channel

	nrf24_WriteRegMulti(TX_ADDR, Address, 5);//To write the Tx Address

	uint8_t config = nrf24_ReadReg(CONFIG);
	config = config | (1<<1);//Adding '1' to 1st position of the CONFIG reg as 1st bit is to power up and 0th bit will be low only as 0~Tx
	nrf24_WriteReg (CONFIG, config);//Powering up the device

	CE_Enable();
}

uint8_t NRF24_Transmit(uint8_t *data)
{
	uint8_t cmdtosend = 0;
	//select the device and send all the configured values
	CE_Enable();

	//paylaod indication command
	cmdtosend = W_TX_PAYLOAD;
	HAL_SPI_Transmit(NRF24_SPI, &cmdtosend, 1, 100);

	//sending payload
	HAL_SPI_Transmit(NRF24_SPI, data, 32, 1000);

	CSN_UnSelect();

	HAL_Delay(1);

	uint8_t fifostatus = nrf24_ReadReg(FIFO_STATUS);
//This if function says weather all the data in the Tx FIFO is sent or not by checking the 4th bit in the "FIFO_STATUS" reg
// and also if the divice is disconnected we all the bits here will get 'set' so using the reserved bits as advantage, we can get to know if the FIFO is really got empty
// or else the device just got disconnected.
	if(fifostatus&(1<<4) && (!(fifostatus&(1<<3))))
	{
		cmdtosend = FLUSH_TX;
		nrfsendcmd(cmdtosend);

		return 1;
	}
	return 0;
}

void NRF24_RxMode (uint8_t *Address, uint8_t channel)
{
	CE_Disable();



		uint8_t en_rxaddr = nrf24_ReadReg(EN_RXADDR);
		en_rxaddr = en_rxaddr | (1<<1);//selecting data pipe 1
		nrf24_WriteReg(EN_RXADDR, en_rxaddr);

		nrf24_WriteRegMulti(RX_ADDR_P1, Address, 5);//To write the Rx Address

		nrf24_WriteReg(RX_PW_P1, 32);//32 bit pay load reception


		uint8_t config = nrf24_ReadReg(CONFIG);
		config = config | (1<<3) | (1<<0);//Adding '1' to 1st position of the CONFIG reg as 1st bit is to power up and 0th bit will be set for Rx mode
		nrf24_WriteReg (CONFIG, config);//Powering up the device

		CE_Enable();
}

uint8_t isDataAvailable (int pipenum)
{
	uint8_t status = nrf24_ReadReg(STATUS);

	if ((status&(1<<6))&&(status&(pipenum<<1)))
	{
		nrf24_WriteReg(STATUS, (1<<6));

		return 1;
	}

	return 0;
}

void NRF24_Receive (uint8_t *data)
{
		uint8_t cmdtosend = 0;
		//select the device and send all the configured values
		CE_Enable();

		//paylaod indication command
		cmdtosend = R_RX_PAYLOAD;
		HAL_SPI_Transmit(NRF24_SPI, &cmdtosend, 1, 100);

		//sending payload
		HAL_SPI_Receive(NRF24_SPI, data, 32, 1000);

		CSN_UnSelect();

		HAL_Delay(1);

		cmdtosend = FLUSH_RX;
		nrfsendcmd(cmdtosend);
}



























