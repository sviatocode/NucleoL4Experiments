/*
 * nRF24L01.h
 *
 *  Created on: Jul 2, 2018
 *      Author: fpgamcu
 */

#ifndef LIBS_NRF24L01P_H_
#define LIBS_NRF24L01P_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#if defined(USE_HAL_DRIVER)

#include "spi.h"
#include "gpio.h"

typedef struct nRF24L01PinStruct
{
    GPIO_TypeDef* port;
    uint16_t pin;

} nRF24L01PinType;

typedef SPI_HandleTypeDef* nRF24L01SpiType;

#define NRF24L01_PIN_SET(PIN,VAL) HAL_GPIO_WritePin(PIN.port,PIN.pin,\
        ((VAL)?GPIO_PIN_SET:GPIO_PIN_RESET))

#define NRF24L01_SPI_WRITE(SPI,BYTES,LEN) HAL_SPI_Transmit(SPI,BYTES,LEN,1000)
#define NRF24L01_SPI_READ(SPI,BYTES,LEN) HAL_SPI_Receive(SPI,BYTES,LEN,1000)

#else

#include "nrf_drv_spi.h"
#include "nrf_gpio.h"

typedef uint32_t nRF24L01PinType;
typedef nrf_drv_spi_t* nRF24L01SpiType;

#define NRF24L01_PIN_SET(PIN,VAL) ((VAL)?nrf_gpio_pin_set(PIN):nrf_gpio_pin_clear(PIN))

#define NRF24L01_SPI_WRITE(SPI,BYTES,LEN) nrf_drv_spi_transfer(SPI,BYTES,LEN,NULL,0)
#define NRF24L01_SPI_READ(SPI,BYTES,LEN) nrf_drv_spi_transfer(SPI,NULL,0,BYTES,LEN)

#endif

/* Interrupt masks */
#define NRF24L01_IRQ_DATA_READY     0x40 /*!< Data ready for receive */
#define NRF24L01_IRQ_TRAN_OK        0x20 /*!< Transmission went OK */
#define NRF24L01_IRQ_MAX_RT         0x10 /*!< Max retransmissions reached, last transmission failed */

/**
 * @brief  Interrupt structure
 */
typedef union nRF24L01IRQUnion {
    struct {
        uint8_t reserved0:4;
        uint8_t MaxRT:1;     /*!< Set to 1 if MAX retransmissions flag is set */
        uint8_t DataSent:1;  /*!< Set to 1 if last transmission is OK */
        uint8_t DataReady:1; /*!< Set to 1 if data are ready to be read */
        uint8_t reserved1:1;
    } F;
    uint8_t Status;          /*!< NRF status register value */
} nRF24L01IRQType;

/**
 * @brief  Transmission status enumeration
 */
typedef enum nRF24L01TransmitStatusEnum {
    nRF24L01_Transmit_Status_Lost = 0x00,   /*!< Message is lost, reached maximum number of retransmissions */
    nRF24L01_Transmit_Status_Ok = 0x01,     /*!< Message sent successfully */
    nRF24L01_Transmit_Status_Sending = 0xFF /*!< Message is still sending */
} nRF24L01TransmitStatusType;

/**
 * @brief  Data rate enumeration
 */
typedef enum nRF24L01DataRateEnum {
    nRF24L01_DataRate_2M = 0x00, /*!< Data rate set to 2Mbps */
    nRF24L01_DataRate_1M,        /*!< Data rate set to 1Mbps */
    nRF24L01_DataRate_250k       /*!< Data rate set to 250kbps */
} nRF24L01DataRateType;

/**
 * @brief  Output power enumeration
 */
typedef enum nRF24L01OutputPowerEnum {
    nRF24L01_OutputPower_M18dBm = 0x00,/*!< Output power set to -18dBm */
    nRF24L01_OutputPower_M12dBm,       /*!< Output power set to -12dBm */
    nRF24L01_OutputPower_M6dBm,        /*!< Output power set to -6dBm */
    nRF24L01_OutputPower_0dBm          /*!< Output power set to 0dBm */
} nRF24L01OutputPowerType;

typedef struct nRF24L01HandleStruct
{
    nRF24L01SpiType hspi;

    nRF24L01PinType csPin;
    nRF24L01PinType cePin;

    uint8_t PayloadSize;                //Payload size
    uint8_t Channel;                    //Channel selected
    nRF24L01OutputPowerType OutPwr;     //Output power
    nRF24L01DataRateType DataRate;      //Data rate

} nRF24L01Handle;

/**
 * @brief  Initializes NRF24L01+ module
 * @param  channel: channel you will use for communication, from 0 to 125 eg. working frequency from 2.4 to 2.525 GHz
 * @param  payload_size: maximum data to be sent in one packet from one NRF to another.
 * @note   Maximal payload size is 32bytes
 * @retval 1
 */
bool nRF24L01_Init(nRF24L01Handle* hnrf, nRF24L01SpiType hspi,
        nRF24L01PinType csPin, nRF24L01PinType cePin, uint8_t channel,
        uint8_t payload_size);

/**
 * @brief  Sets own address. This is used for settings own id when communication with other modules
 * @note   "Own" address of one device must be the same as "TX" address of other device (and vice versa),
 *         if you want to get successful communication
 * @param  *adr: Pointer to 5-bytes length array with address
 * @retval None
 */
void nRF24L01_SetMyAddress(nRF24L01Handle* hnrf, uint8_t *adr);

/**
 * @brief  Sets address you will communicate with
 * @note   "Own" address of one device must be the same as "TX" address of other device (and vice versa),
 *         if you want to get successful communication
 * @param  *adr: Pointer to 5-bytes length array with address
 * @retval None
 */
void nRF24L01_SetTxAddress(nRF24L01Handle* hnrf, uint8_t *adr);

/**
 * @brief  Gets number of retransmissions needed in last transmission
 * @param  None
 * @retval Number of retransmissions, between 0 and 15.
 */
uint8_t nRF24L01_GetRetransmissionsCount(nRF24L01Handle* hnrf);

/**
 * @brief  Sets NRF24L01+ to TX mode
 * @note   In this mode is NRF able to send data to another NRF module
 * @param  None
 * @retval None
 */
void nRF24L01_PowerUpTx(nRF24L01Handle* hnrf);

/**
 * @brief  Sets NRF24L01+ to RX mode
 * @note   In this mode is NRF able to receive data from another NRF module.
 *         This is default mode and should be used all the time, except when sending data
 * @param  None
 * @retval None
 */
void nRF24L01_PowerUpRx(nRF24L01Handle* hnrf);

/**
 * @brief  Sets NRF24L01+ to power down mode
 * @note   In power down mode, you are not able to transmit/receive data.
 *         You can wake up device using @ref TM_NRF24L01_PowerUpTx() or @ref TM_NRF24L01_PowerUpRx() functions
 * @param  None
 * @retval None
 */
void nRF24L01_PowerDown(nRF24L01Handle* hnrf);

/**
 * @brief  Gets transmissions status
 * @param  None
 * @retval Transmission status. Return is based on @ref TM_NRF24L01_Transmit_Status_t enumeration
 */
nRF24L01TransmitStatusType nRF24L01_GetTransmissionStatus(nRF24L01Handle* hnrf);

/**
 * @brief  Transmits data with NRF24L01+ to another NRF module
 * @param  *data: Pointer to 8-bit array with data.
 *         Maximum length of array can be the same as "payload_size" parameter on initialization
 * @retval None
 */
void nRF24L01_Transmit(nRF24L01Handle* hnrf, uint8_t *data);

/**
 * @brief  Checks if data is ready to be read from NRF24L01+
 * @param  None
 * @retval Data ready status:
 *            - 0: No data available for receive in bufferReturns
 *            - > 0: Data is ready to be collected
 */
uint8_t nRF24L01_DataReady(nRF24L01Handle* hnrf);

/**
 * @brief  Gets data from NRF24L01+
 * @param  *data: Pointer to 8-bits array where data from NRF will be saved
 * @retval None
 */
void nRF24L01_GetData(nRF24L01Handle* hnrf, uint8_t* data);

/**
 * @brief  Sets working channel
 * @note   Channel value is just an offset in units MHz from 2.4GHz
 *         For example, if you select channel 65, then operation frequency will be set to 2.465GHz.
 * @param  channel: RF channel where device will operate
 * @retval None
 */
void nRF24L01_SetChannel(nRF24L01Handle* hnrf, uint8_t channel);

/**
 * @brief  Sets RF parameters for NRF24L01+
 * @param  DataRate: Data rate selection for NRF module. This parameter can be a value of @ref TM_NRF24L01_DataRate_t enumeration
 * @param  OutPwr: Output power selection for NRF module. This parameter can be a value of @ref TM_NRF24L01_OutputPower_t enumeration
 * @retval None
 */
void nRF24L01_SetRF(nRF24L01Handle* hnrf, nRF24L01DataRateType DataRate,
        nRF24L01OutputPowerType OutPwr);

/**
 * @brief  Gets NRLF+ status register value
 * @param  None
 * @retval Status register from NRF
 */
uint8_t nRF24L01_GetStatus(nRF24L01Handle* hnrf);

/**
 * @brief  Reads interrupts from NRF
 * @param  *IRQ: Pointer to @ref TM_NRF24L01_IRQ_t where IRQ status will be saved
 * @retval IRQ status
 *            - 0: No interrupts are active
 *            - > 0: At least one interrupt is active
 */
uint8_t nRF24L01_Read_Interrupts(nRF24L01Handle* hnrf,
        nRF24L01IRQType* IRQ);

/**
 * @brief  Clears interrupt status
 * @param  None
 * @retval None
 */
void nRF24L01_Clear_Interrupts(nRF24L01Handle* hnrf);

#endif /* LIBS_NRF24L01P_H_ */
