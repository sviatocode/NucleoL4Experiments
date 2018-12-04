/*
 * nrfWorks.c
 *
 *  Created on: Aug 30, 2018
 *      Author: fpgam
 */

#include <string.h>
#include "nRF24L01p.h"

nRF24L01Handle hnrf1;
nRF24L01Handle hnrf2;

/* My address */
uint8_t MyAddress[] = {
    0xE7,
    0xE7,
    0xE7,
    0xE7,
    0xE7
};
/* Receiver address */
uint8_t TxAddress[] = {
    1,
    2,
    3,
    4,
    5
};

void RxInit(void)
{
    nRF24L01_Init(&hnrf2, &hspi1,
            (nRF24L01PinType){ port:RF2_CS_GPIO_Port, pin:RF2_CS_Pin },
            (nRF24L01PinType){ port:RF2_CE_GPIO_Port, pin:RF2_CE_Pin }, 15, 32);

    /* Set 2MBps data rate and -18dBm output power */
    nRF24L01_SetRF(&hnrf2, nRF24L01_DataRate_2M, nRF24L01_OutputPower_M18dBm);

    /* Set my address, 5 bytes */
    nRF24L01_SetMyAddress(&hnrf2, TxAddress);

    /* Set TX address, 5 bytes */
    nRF24L01_SetTxAddress(&hnrf2, MyAddress);
}

void RxProcess(void)
{
    /* Data received */
    uint8_t dataIn[32];

    /* NRF transmission status */
    nRF24L01TransmitStatusType transmissionStatus;

    if (nRF24L01_DataReady(&hnrf2)) {
        /* Get data from NRF24L01+ */
        nRF24L01_GetData(&hnrf2, dataIn);

        /* Start send */
        HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);

        /* Send it back, automatically goes to TX mode */
        nRF24L01_Transmit(&hnrf2, dataIn);

        /* Wait for data to be sent */
        do {
            /* Wait till sending */
            transmissionStatus = nRF24L01_GetTransmissionStatus(&hnrf2);
        } while (transmissionStatus == nRF24L01_Transmit_Status_Sending);

        /* Send done */
        HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

        /* Go back to RX mode */
        nRF24L01_PowerUpRx(&hnrf2);
    }
}

void TxInit(void)
{
    /* Initialize NRF24L01+ on channel 15 and 32bytes of payload */
    /* By default 2Mbps data rate and 0dBm output power */
    /* NRF24L01 goes to RX mode by default */
    nRF24L01_Init(&hnrf1, &hspi1,
          (nRF24L01PinType){ port:RF1_CS_GPIO_Port, pin:RF1_CS_Pin },
          (nRF24L01PinType){ port:RF1_CE_GPIO_Port, pin:RF1_CE_Pin }, 15, 32);

    /* Set 2MBps data rate and -18dBm output power */
    nRF24L01_SetRF(&hnrf1, nRF24L01_DataRate_2M, nRF24L01_OutputPower_M18dBm);

    /* Set my address, 5 bytes */
    nRF24L01_SetMyAddress(&hnrf1, MyAddress);

    /* Set TX address, 5 bytes */
    nRF24L01_SetTxAddress(&hnrf1, TxAddress);
}

void TxProcess(void)
{
    /* Data received and data for send */
     uint8_t dataOut[32], dataIn[32];

    /* NRF transmission status */
     nRF24L01TransmitStatusType transmissionStatus;

    /* Buffer for strings */
    char str[40];

    /* Fill data with something */
    sprintf((char *)dataOut, "abcdefghijklmnoszxABCDEFCBDA");

    /* Display on USART */
    printf("pinging: ");

    /* Transmit data, goes automatically to TX mode */
    nRF24L01_Transmit(&hnrf1, dataOut);

    /* Turn on led to indicate sending */
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);

    /* Wait for data to be sent */
    do {
        /* Get transmission status */
        transmissionStatus = nRF24L01_GetTransmissionStatus(&hnrf1);
//        RxProcess();
    } while (transmissionStatus == nRF24L01_Transmit_Status_Sending);

    /* Turn off led */
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

    /* Go back to RX mode */
    nRF24L01_PowerUpRx(&hnrf1);

    /* Wait received data, wait max 100ms, if time is larger, then data were probably lost */
    uint32_t start = HAL_GetTick();
    while (!nRF24L01_DataReady(&hnrf1) && ((HAL_GetTick() - start) < 100));

    /* Format time */
    sprintf(str, "%ld ms", (HAL_GetTick() - start));

    /* Show ping time */
    printf(str);

    /* Get data from NRF2L01+ */
    nRF24L01_GetData(&hnrf1, dataIn);

    /* Check transmit status */
    if (transmissionStatus == nRF24L01_Transmit_Status_Ok) {
        /* Transmit went OK */
        printf(": OK\n");
    } else if (transmissionStatus == nRF24L01_Transmit_Status_Lost) {
        /* Message was LOST */
        printf(": LOST\n");
    } else {
        /* This should never happen */
        printf(": SENDING\n");
    }
}

