/*
 * nRF24L01.c
 *
 *  Created on: Jul 2, 2018
 *      Author: fpgamcu
 */

#include "nRF24L01p.h"

/* NRF24L01+ registers*/
#define NRF24L01_REG_CONFIG         0x00    //Configuration Register
#define NRF24L01_REG_EN_AA          0x01    //Enable ‘Auto Acknowledgment’ Function
#define NRF24L01_REG_EN_RXADDR      0x02    //Enabled RX Addresses
#define NRF24L01_REG_SETUP_AW       0x03    //Setup of Address Widths (common for all data pipes)
#define NRF24L01_REG_SETUP_RETR     0x04    //Setup of Automatic Retransmission
#define NRF24L01_REG_RF_CH          0x05    //RF Channel
#define NRF24L01_REG_RF_SETUP       0x06    //RF Setup Register
#define NRF24L01_REG_STATUS         0x07    //Status Register
#define NRF24L01_REG_OBSERVE_TX     0x08    //Transmit observe registerf
#define NRF24L01_REG_RPD            0x09
#define NRF24L01_REG_RX_ADDR_P0     0x0A    //Receive address data pipe 0. 5 Bytes maximum length.
#define NRF24L01_REG_RX_ADDR_P1     0x0B    //Receive address data pipe 1. 5 Bytes maximum length.
#define NRF24L01_REG_RX_ADDR_P2     0x0C    //Receive address data pipe 2. Only LSB
#define NRF24L01_REG_RX_ADDR_P3     0x0D    //Receive address data pipe 3. Only LSB
#define NRF24L01_REG_RX_ADDR_P4     0x0E    //Receive address data pipe 4. Only LSB
#define NRF24L01_REG_RX_ADDR_P5     0x0F    //Receive address data pipe 5. Only LSB
#define NRF24L01_REG_TX_ADDR        0x10    //Transmit address. Used for a PTX device only
#define NRF24L01_REG_RX_PW_P0       0x11
#define NRF24L01_REG_RX_PW_P1       0x12
#define NRF24L01_REG_RX_PW_P2       0x13
#define NRF24L01_REG_RX_PW_P3       0x14
#define NRF24L01_REG_RX_PW_P4       0x15
#define NRF24L01_REG_RX_PW_P5       0x16
#define NRF24L01_REG_FIFO_STATUS    0x17    //FIFO Status Register
#define NRF24L01_REG_DYNPD          0x1C    //Enable dynamic payload length
#define NRF24L01_REG_FEATURE        0x1D

/* Registers default values */
#define NRF24L01_REG_DEFAULT_VAL_CONFIG         0x08
#define NRF24L01_REG_DEFAULT_VAL_EN_AA          0x3F
#define NRF24L01_REG_DEFAULT_VAL_EN_RXADDR      0x03
#define NRF24L01_REG_DEFAULT_VAL_SETUP_AW       0x03
#define NRF24L01_REG_DEFAULT_VAL_SETUP_RETR     0x03
#define NRF24L01_REG_DEFAULT_VAL_RF_CH          0x02
#define NRF24L01_REG_DEFAULT_VAL_RF_SETUP       0x0E
#define NRF24L01_REG_DEFAULT_VAL_STATUS         0x0E
#define NRF24L01_REG_DEFAULT_VAL_OBSERVE_TX     0x00
#define NRF24L01_REG_DEFAULT_VAL_RPD            0x00
#define NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P0_0   0xE7
#define NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P0_1   0xE7
#define NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P0_2   0xE7
#define NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P0_3   0xE7
#define NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P0_4   0xE7
#define NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P1_0   0xC2
#define NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P1_1   0xC2
#define NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P1_2   0xC2
#define NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P1_3   0xC2
#define NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P1_4   0xC2
#define NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P2     0xC3
#define NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P3     0xC4
#define NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P4     0xC5
#define NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P5     0xC6
#define NRF24L01_REG_DEFAULT_VAL_TX_ADDR_0      0xE7
#define NRF24L01_REG_DEFAULT_VAL_TX_ADDR_1      0xE7
#define NRF24L01_REG_DEFAULT_VAL_TX_ADDR_2      0xE7
#define NRF24L01_REG_DEFAULT_VAL_TX_ADDR_3      0xE7
#define NRF24L01_REG_DEFAULT_VAL_TX_ADDR_4      0xE7
#define NRF24L01_REG_DEFAULT_VAL_RX_PW_P0       0x00
#define NRF24L01_REG_DEFAULT_VAL_RX_PW_P1       0x00
#define NRF24L01_REG_DEFAULT_VAL_RX_PW_P2       0x00
#define NRF24L01_REG_DEFAULT_VAL_RX_PW_P3       0x00
#define NRF24L01_REG_DEFAULT_VAL_RX_PW_P4       0x00
#define NRF24L01_REG_DEFAULT_VAL_RX_PW_P5       0x00
#define NRF24L01_REG_DEFAULT_VAL_FIFO_STATUS    0x11
#define NRF24L01_REG_DEFAULT_VAL_DYNPD          0x00
#define NRF24L01_REG_DEFAULT_VAL_FEATURE        0x00

/* Configuration register*/
#define NRF24L01_MASK_RX_DR     6
#define NRF24L01_MASK_TX_DS     5
#define NRF24L01_MASK_MAX_RT    4
#define NRF24L01_EN_CRC         3
#define NRF24L01_CRCO           2
#define NRF24L01_PWR_UP         1
#define NRF24L01_PRIM_RX        0

/* Enable auto acknowledgment*/
#define NRF24L01_ENAA_P5        5
#define NRF24L01_ENAA_P4        4
#define NRF24L01_ENAA_P3        3
#define NRF24L01_ENAA_P2        2
#define NRF24L01_ENAA_P1        1
#define NRF24L01_ENAA_P0        0

/* Enable rx addresses */
#define NRF24L01_ERX_P5         5
#define NRF24L01_ERX_P4         4
#define NRF24L01_ERX_P3         3
#define NRF24L01_ERX_P2         2
#define NRF24L01_ERX_P1         1
#define NRF24L01_ERX_P0         0

/* Setup of address width */
#define NRF24L01_AW             0 //2 bits

/* Setup of auto re-transmission*/
#define NRF24L01_ARD            4 //4 bits
#define NRF24L01_ARC            0 //4 bits

/* RF setup register*/
#define NRF24L01_PLL_LOCK       4
#define NRF24L01_RF_DR_LOW      5
#define NRF24L01_RF_DR_HIGH     3
#define NRF24L01_RF_DR          3
#define NRF24L01_RF_PWR         1 //2 bits

/* General status register */
#define NRF24L01_RX_DR          6
#define NRF24L01_TX_DS          5
#define NRF24L01_MAX_RT         4
#define NRF24L01_RX_P_NO        1 //3 bits
#define NRF24L01_TX_FULL        0

/* Transmit observe register */
#define NRF24L01_PLOS_CNT       4 //4 bits
#define NRF24L01_ARC_CNT        0 //4 bits

/* FIFO status*/
#define NRF24L01_TX_REUSE       6
#define NRF24L01_FIFO_FULL      5
#define NRF24L01_TX_EMPTY       4
#define NRF24L01_RX_FULL        1
#define NRF24L01_RX_EMPTY       0

//Dynamic length
#define NRF24L01_DPL_P0         0
#define NRF24L01_DPL_P1         1
#define NRF24L01_DPL_P2         2
#define NRF24L01_DPL_P3         3
#define NRF24L01_DPL_P4         4
#define NRF24L01_DPL_P5         5

/* Transmitter power*/
#define NRF24L01_M18DBM         0 //-18 dBm
#define NRF24L01_M12DBM         1 //-12 dBm
#define NRF24L01_M6DBM          2 //-6 dBm
#define NRF24L01_0DBM           3 //0 dBm

/* Data rates */
#define NRF24L01_2MBPS          0
#define NRF24L01_1MBPS          1
#define NRF24L01_250KBPS        2

/* Configuration */
#define NRF24L01_CONFIG         ((1 << NRF24L01_EN_CRC) | (0 << NRF24L01_CRCO))

/* Instruction Mnemonics */
#define NRF24L01_REGISTER_MASK              0x1F

#define NRF24L01_READ_REGISTER_MASK(reg)    (0x00 | (NRF24L01_REGISTER_MASK & reg)) //Last 5 bits will indicate reg. address
#define NRF24L01_WRITE_REGISTER_MASK(reg)   (0x20 | (NRF24L01_REGISTER_MASK & reg)) //Last 5 bits will indicate reg. address
#define NRF24L01_R_RX_PAYLOAD_MASK          0x61
#define NRF24L01_W_TX_PAYLOAD_MASK          0xA0
#define NRF24L01_FLUSH_TX_MASK              0xE1
#define NRF24L01_FLUSH_RX_MASK              0xE2
#define NRF24L01_REUSE_TX_PL_MASK           0xE3
#define NRF24L01_ACTIVATE_MASK              0x50
#define NRF24L01_R_RX_PL_WID_MASK           0x60
#define NRF24L01_NOP_MASK                   0xFF

#define NRF24L01_TRANSMISSON_OK             0
#define NRF24L01_MESSAGE_LOST               1

#define NRF24L01_CHECK_BIT(reg, bit)       (reg & (1 << bit))

static void nRF24L01_FlushTx(nRF24L01Handle* hnrf)
{
    uint8_t cmd = NRF24L01_FLUSH_TX_MASK;
    NRF24L01_PIN_SET(hnrf->csPin, 0);
    NRF24L01_SPI_WRITE(hnrf->hspi, &cmd, 1);
    NRF24L01_PIN_SET(hnrf->csPin, 1);
}

static void nRF24L01_FlushRx(nRF24L01Handle* hnrf)
{
    uint8_t cmd = NRF24L01_FLUSH_RX_MASK;
    NRF24L01_PIN_SET(hnrf->csPin, 0);
    NRF24L01_SPI_WRITE(hnrf->hspi, &cmd, 1);
    NRF24L01_PIN_SET(hnrf->csPin, 1);
}

static void nRF24L01_WriteRegisterMulti(nRF24L01Handle* hnrf, uint8_t reg,
        uint8_t *data, uint8_t count)
{
    uint8_t cmd = NRF24L01_WRITE_REGISTER_MASK(reg);
    NRF24L01_PIN_SET(hnrf->csPin, 0);
    NRF24L01_SPI_WRITE(hnrf->hspi, &cmd, 1);
    NRF24L01_SPI_WRITE(hnrf->hspi, data, count);
    NRF24L01_PIN_SET(hnrf->csPin, 1);
}

static void nRF24L01_WriteRegister(nRF24L01Handle* hnrf, uint8_t reg, uint8_t value)
{
    nRF24L01_WriteRegisterMulti(hnrf, reg, &value, 1);
}

static void nRF24L01_SoftReset(nRF24L01Handle* hnrf)
{
    uint8_t data[5];

    nRF24L01_WriteRegister(hnrf, NRF24L01_REG_CONFIG,      NRF24L01_REG_DEFAULT_VAL_CONFIG);
    nRF24L01_WriteRegister(hnrf, NRF24L01_REG_EN_AA,       NRF24L01_REG_DEFAULT_VAL_EN_AA);
    nRF24L01_WriteRegister(hnrf, NRF24L01_REG_EN_RXADDR,   NRF24L01_REG_DEFAULT_VAL_EN_RXADDR);
    nRF24L01_WriteRegister(hnrf, NRF24L01_REG_SETUP_AW,    NRF24L01_REG_DEFAULT_VAL_SETUP_AW);
    nRF24L01_WriteRegister(hnrf, NRF24L01_REG_SETUP_RETR,  NRF24L01_REG_DEFAULT_VAL_SETUP_RETR);
    nRF24L01_WriteRegister(hnrf, NRF24L01_REG_RF_CH,       NRF24L01_REG_DEFAULT_VAL_RF_CH);
    nRF24L01_WriteRegister(hnrf, NRF24L01_REG_RF_SETUP,    NRF24L01_REG_DEFAULT_VAL_RF_SETUP);
    nRF24L01_WriteRegister(hnrf, NRF24L01_REG_STATUS,      NRF24L01_REG_DEFAULT_VAL_STATUS);
    nRF24L01_WriteRegister(hnrf, NRF24L01_REG_OBSERVE_TX,  NRF24L01_REG_DEFAULT_VAL_OBSERVE_TX);
    nRF24L01_WriteRegister(hnrf, NRF24L01_REG_RPD,         NRF24L01_REG_DEFAULT_VAL_RPD);

    //P0
    data[0] = NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P0_0;
    data[1] = NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P0_1;
    data[2] = NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P0_2;
    data[3] = NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P0_3;
    data[4] = NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P0_4;
    nRF24L01_WriteRegisterMulti(hnrf, NRF24L01_REG_RX_ADDR_P0, data, 5);

    //P1
    data[0] = NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P1_0;
    data[1] = NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P1_1;
    data[2] = NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P1_2;
    data[3] = NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P1_3;
    data[4] = NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P1_4;
    nRF24L01_WriteRegisterMulti(hnrf, NRF24L01_REG_RX_ADDR_P1, data, 5);

    nRF24L01_WriteRegister(hnrf, NRF24L01_REG_RX_ADDR_P2,  NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P2);
    nRF24L01_WriteRegister(hnrf, NRF24L01_REG_RX_ADDR_P3,  NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P3);
    nRF24L01_WriteRegister(hnrf, NRF24L01_REG_RX_ADDR_P4,  NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P4);
    nRF24L01_WriteRegister(hnrf, NRF24L01_REG_RX_ADDR_P5,  NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P5);

    //TX
    data[0] = NRF24L01_REG_DEFAULT_VAL_TX_ADDR_0;
    data[1] = NRF24L01_REG_DEFAULT_VAL_TX_ADDR_1;
    data[2] = NRF24L01_REG_DEFAULT_VAL_TX_ADDR_2;
    data[3] = NRF24L01_REG_DEFAULT_VAL_TX_ADDR_3;
    data[4] = NRF24L01_REG_DEFAULT_VAL_TX_ADDR_4;
    nRF24L01_WriteRegisterMulti(hnrf, NRF24L01_REG_TX_ADDR, data, 5);

    nRF24L01_WriteRegister(hnrf, NRF24L01_REG_RX_PW_P0,    NRF24L01_REG_DEFAULT_VAL_RX_PW_P0);
    nRF24L01_WriteRegister(hnrf, NRF24L01_REG_RX_PW_P1,    NRF24L01_REG_DEFAULT_VAL_RX_PW_P1);
    nRF24L01_WriteRegister(hnrf, NRF24L01_REG_RX_PW_P2,    NRF24L01_REG_DEFAULT_VAL_RX_PW_P2);
    nRF24L01_WriteRegister(hnrf, NRF24L01_REG_RX_PW_P3,    NRF24L01_REG_DEFAULT_VAL_RX_PW_P3);
    nRF24L01_WriteRegister(hnrf, NRF24L01_REG_RX_PW_P4,    NRF24L01_REG_DEFAULT_VAL_RX_PW_P4);
    nRF24L01_WriteRegister(hnrf, NRF24L01_REG_RX_PW_P5,    NRF24L01_REG_DEFAULT_VAL_RX_PW_P5);
    nRF24L01_WriteRegister(hnrf, NRF24L01_REG_FIFO_STATUS, NRF24L01_REG_DEFAULT_VAL_FIFO_STATUS);
    nRF24L01_WriteRegister(hnrf, NRF24L01_REG_DYNPD,       NRF24L01_REG_DEFAULT_VAL_DYNPD);
    nRF24L01_WriteRegister(hnrf, NRF24L01_REG_FEATURE,     NRF24L01_REG_DEFAULT_VAL_FEATURE);
}

static void nRF24L01_ReadRegisterMulti(nRF24L01Handle* hnrf, uint8_t reg,
        uint8_t* data, uint8_t count)
{
    uint8_t cmd = NRF24L01_READ_REGISTER_MASK(reg);
    NRF24L01_PIN_SET(hnrf->csPin, 0);
    NRF24L01_SPI_WRITE(hnrf->hspi, &cmd, 1);
    NRF24L01_SPI_READ(hnrf->hspi, data, count); // TODO check why NRF24L01_NOP_MASK was used
    NRF24L01_PIN_SET(hnrf->csPin, 1);
}

static uint8_t nRF24L01_ReadRegister(nRF24L01Handle* hnrf, uint8_t reg)
{
    uint8_t value;
    nRF24L01_ReadRegisterMulti(hnrf, reg, &value, 1);
    return value;
}

static void nRF24L01_WriteBit(nRF24L01Handle* hnrf, uint8_t reg, uint8_t bit,
        uint8_t value)
{
    uint8_t tmp;
    /* Read register */
    tmp = nRF24L01_ReadRegister(hnrf, reg);
    /* Make operation */
    if (value) {
        tmp |= 1 << bit;
    } else {
        tmp &= ~(1 << bit);
    }
    /* Write back */
    nRF24L01_WriteRegister(hnrf, reg, tmp);
}

static uint8_t nRF24L01_RxFifoEmpty(nRF24L01Handle* hnrf)
{
    uint8_t reg = nRF24L01_ReadRegister(hnrf, NRF24L01_REG_FIFO_STATUS);
    return NRF24L01_CHECK_BIT(reg, NRF24L01_RX_EMPTY);
}

//static uint8_t nRF24L01_ReadBit(nRF24L01Handle* hnrf, uint8_t reg, uint8_t bit)
//{
//    uint8_t tmp;
//    tmp = nRF24L01_ReadRegister(hnrf, reg);
//    if (!NRF24L01_CHECK_BIT(tmp, bit)) {
//        return 0;
//    }
//    return 1;
//}

bool nRF24L01_Init(nRF24L01Handle* hnrf, nRF24L01SpiType hspi,
        nRF24L01PinType csPin, nRF24L01PinType cePin, uint8_t channel,
        uint8_t payload_size)
{
    /* Initialize handle fields */
    hnrf->hspi = hspi;
    hnrf->csPin = csPin;
    hnrf->cePin = cePin;

    /* Initialize CE and CSN pins */
    NRF24L01_PIN_SET(hnrf->csPin, 1);
    NRF24L01_PIN_SET(hnrf->cePin, 0);

    /* Max payload is 32bytes */
    if (payload_size > 32) {
        payload_size = 32;
    }

    /* Fill structure */
    hnrf->Channel = !channel; /* Set channel to some different value for TM_NRF24L01_SetChannel() function */
    hnrf->PayloadSize = payload_size;
    hnrf->OutPwr = nRF24L01_OutputPower_0dBm;
    hnrf->DataRate = nRF24L01_DataRate_2M;

    /* Reset nRF24L01+ to power on registers values */
    nRF24L01_SoftReset(hnrf);

    /* Channel select */
    nRF24L01_SetChannel(hnrf, channel);

    /* Set pipeline to max possible 32 bytes */
    nRF24L01_WriteRegister(hnrf, NRF24L01_REG_RX_PW_P0, hnrf->PayloadSize); // Auto-ACK pipe
    nRF24L01_WriteRegister(hnrf, NRF24L01_REG_RX_PW_P1, hnrf->PayloadSize); // Data payload pipe
    nRF24L01_WriteRegister(hnrf, NRF24L01_REG_RX_PW_P2, hnrf->PayloadSize);
    nRF24L01_WriteRegister(hnrf, NRF24L01_REG_RX_PW_P3, hnrf->PayloadSize);
    nRF24L01_WriteRegister(hnrf, NRF24L01_REG_RX_PW_P4, hnrf->PayloadSize);
    nRF24L01_WriteRegister(hnrf, NRF24L01_REG_RX_PW_P5, hnrf->PayloadSize);

    /* Set RF settings (2mbps, output power) */
    nRF24L01_SetRF(hnrf, hnrf->DataRate, hnrf->OutPwr);

    /* Config register */
    nRF24L01_WriteRegister(hnrf, NRF24L01_REG_CONFIG, NRF24L01_CONFIG);

    /* Enable auto-acknowledgment for all pipes */
    nRF24L01_WriteRegister(hnrf, NRF24L01_REG_EN_AA, 0x3F);

    /* Enable RX addresses */
    nRF24L01_WriteRegister(hnrf, NRF24L01_REG_EN_RXADDR, 0x3F);

    /* Auto retransmit delay: 1000 (4x250) us and Up to 15 retransmit trials */
    nRF24L01_WriteRegister(hnrf, NRF24L01_REG_SETUP_RETR, 0x4F);

    /* Dynamic length configurations: No dynamic length */
    nRF24L01_WriteRegister(hnrf, NRF24L01_REG_DYNPD, (0 << NRF24L01_DPL_P0) | (0 << NRF24L01_DPL_P1) | (0 << NRF24L01_DPL_P2) | (0 << NRF24L01_DPL_P3) | (0 << NRF24L01_DPL_P4) | (0 << NRF24L01_DPL_P5));

    /* Clear FIFOs */
    nRF24L01_FlushTx(hnrf);
    nRF24L01_FlushRx(hnrf);

    /* Clear interrupts */
    nRF24L01_Clear_Interrupts(hnrf);

    /* Go to RX mode */
    nRF24L01_PowerUpRx(hnrf);

    /* Return OK */
    return true;
}

void nRF24L01_SetMyAddress(nRF24L01Handle* hnrf, uint8_t *adr)
{
    NRF24L01_PIN_SET(hnrf->cePin, 0);
    nRF24L01_WriteRegisterMulti(hnrf, NRF24L01_REG_RX_ADDR_P1, adr, 5);
    NRF24L01_PIN_SET(hnrf->cePin, 1);
}

void nRF24L01_SetTxAddress(nRF24L01Handle* hnrf, uint8_t *adr)
{
    nRF24L01_WriteRegisterMulti(hnrf, NRF24L01_REG_RX_ADDR_P0, adr, 5);
    nRF24L01_WriteRegisterMulti(hnrf, NRF24L01_REG_TX_ADDR, adr, 5);
}

uint8_t nRF24L01_GetRetransmissionsCount(nRF24L01Handle* hnrf)
{
    /* Low 4 bits */
    return nRF24L01_ReadRegister(hnrf, NRF24L01_REG_OBSERVE_TX) & 0x0F;
}

void nRF24L01_PowerUpTx(nRF24L01Handle* hnrf)
{
    nRF24L01_Clear_Interrupts(hnrf);
    nRF24L01_WriteRegister(hnrf, NRF24L01_REG_CONFIG, NRF24L01_CONFIG |
            (0 << NRF24L01_PRIM_RX) | (1 << NRF24L01_PWR_UP));
}

void nRF24L01_PowerUpRx(nRF24L01Handle* hnrf)
{
    /* Disable RX/TX mode */
    NRF24L01_PIN_SET(hnrf->cePin, 0);
    /* Clear RX buffer */
    nRF24L01_FlushRx(hnrf);
    /* Clear interrupts */
    nRF24L01_Clear_Interrupts(hnrf);
    /* Setup RX mode */
    nRF24L01_WriteRegister(hnrf, NRF24L01_REG_CONFIG,
            NRF24L01_CONFIG | 1 << NRF24L01_PWR_UP | 1 << NRF24L01_PRIM_RX);
    /* Start listening */
    NRF24L01_PIN_SET(hnrf->cePin, 1);
}

void nRF24L01_PowerDown(nRF24L01Handle* hnrf)
{
    NRF24L01_PIN_SET(hnrf->cePin, 0);
    nRF24L01_WriteBit(hnrf, NRF24L01_REG_CONFIG, NRF24L01_PWR_UP, 0);
}

nRF24L01TransmitStatusType nRF24L01_GetTransmissionStatus(nRF24L01Handle* hnrf)
{
    uint8_t status = nRF24L01_GetStatus(hnrf);
    if (NRF24L01_CHECK_BIT(status, NRF24L01_TX_DS)) {
        /* Successfully sent */
        return nRF24L01_Transmit_Status_Ok;
    } else if (NRF24L01_CHECK_BIT(status, NRF24L01_MAX_RT)) {
        /* Message lost */
        return nRF24L01_Transmit_Status_Lost;
    }

    /* Still sending */
    return nRF24L01_Transmit_Status_Sending;
}

void nRF24L01_Transmit(nRF24L01Handle* hnrf, uint8_t *data)
{
    uint8_t count = hnrf->PayloadSize;

    /* Chip enable put to low, disable it */
    NRF24L01_PIN_SET(hnrf->cePin, 0);

    /* Go to power up tx mode */
    nRF24L01_PowerUpTx(hnrf);

    /* Clear TX FIFO from NRF24L01+ */
    nRF24L01_FlushTx(hnrf);

    uint8_t cmd = NRF24L01_W_TX_PAYLOAD_MASK;

    /* Send payload to nRF24L01+ */
    NRF24L01_PIN_SET(hnrf->csPin, 0);
    /* Send write payload command */
    NRF24L01_SPI_WRITE(hnrf->hspi, &cmd, 1);
    /* Fill payload with data*/
    NRF24L01_SPI_WRITE(hnrf->hspi, data, count);
    /* Disable SPI */
    NRF24L01_PIN_SET(hnrf->csPin, 1);

    /* Send data! */
    NRF24L01_PIN_SET(hnrf->cePin, 1);
}

uint8_t nRF24L01_DataReady(nRF24L01Handle* hnrf)
{
    uint8_t status = nRF24L01_GetStatus(hnrf);

    if (NRF24L01_CHECK_BIT(status, NRF24L01_RX_DR)) {
        return 1;
    }
    return !nRF24L01_RxFifoEmpty(hnrf);
}

void nRF24L01_GetData(nRF24L01Handle* hnrf, uint8_t* data)
{
//    /* Pull down chip select */
//    NRF24L01_CSN_LOW;
//    /* Send read payload command*/
//    TM_SPI_Send(NRF24L01_SPI, NRF24L01_R_RX_PAYLOAD_MASK);
//    /* Read payload */
//    TM_SPI_SendMulti(NRF24L01_SPI, data, data, TM_NRF24L01_Struct.PayloadSize);
//    /* Pull up chip select */
//    NRF24L01_CSN_HIGH;

    uint8_t cmd = NRF24L01_R_RX_PAYLOAD_MASK;
    NRF24L01_PIN_SET(hnrf->csPin, 0);
    NRF24L01_SPI_WRITE(hnrf->hspi, &cmd, 1);
    NRF24L01_SPI_READ(hnrf->hspi, data, hnrf->PayloadSize);
    NRF24L01_PIN_SET(hnrf->csPin, 1);

    /* Reset status register, clear RX_DR interrupt flag */
    nRF24L01_WriteRegister(hnrf, NRF24L01_REG_STATUS, (1 << NRF24L01_RX_DR));
}

void nRF24L01_SetChannel(nRF24L01Handle* hnrf, uint8_t channel)
{
    if (channel <= 125 && channel != hnrf->Channel) {
        /* Store new channel setting */
        hnrf->Channel = channel;
        /* Write channel */
        nRF24L01_WriteRegister(hnrf, NRF24L01_REG_RF_CH, channel);
    }
}

void nRF24L01_SetRF(nRF24L01Handle* hnrf, nRF24L01DataRateType DataRate,
        nRF24L01OutputPowerType OutPwr)
{
    uint8_t tmp = 0;
    hnrf->DataRate = DataRate;
    hnrf->OutPwr = OutPwr;

    if (DataRate == nRF24L01_DataRate_2M) {
        tmp |= 1 << NRF24L01_RF_DR_HIGH;
    } else if (DataRate == nRF24L01_DataRate_250k) {
        tmp |= 1 << NRF24L01_RF_DR_LOW;
    }
    /* If 1Mbps, all bits set to 0 */

    if (OutPwr == nRF24L01_OutputPower_0dBm) {
        tmp |= 3 << NRF24L01_RF_PWR;
    } else if (OutPwr == nRF24L01_OutputPower_M6dBm) {
        tmp |= 2 << NRF24L01_RF_PWR;
    } else if (OutPwr == nRF24L01_OutputPower_M12dBm) {
        tmp |= 1 << NRF24L01_RF_PWR;
    }

    nRF24L01_WriteRegister(hnrf, NRF24L01_REG_RF_SETUP, tmp);
}

uint8_t nRF24L01_GetStatus(nRF24L01Handle* hnrf)
{
//    uint8_t status;

//    NRF24L01_PIN_SET(hnrf->csPin, 0);
//    /* First received byte is always status register */
//    NRF24L01_SPI_READWRITE(hnrf->hspi, &status, 1, NRF24L01_NOP_MASK, 1);
//    /* Pull up chip select */
//    NRF24L01_PIN_SET(hnrf->csPin, 1);

//    return status;
    return nRF24L01_ReadRegister(hnrf, NRF24L01_REG_STATUS);
}

uint8_t nRF24L01_Read_Interrupts(nRF24L01Handle* hnrf,
        nRF24L01IRQType* IRQ)
{
    IRQ->Status = nRF24L01_GetStatus(hnrf);
    return IRQ->Status;
}

void nRF24L01_Clear_Interrupts(nRF24L01Handle* hnrf)
{
    nRF24L01_WriteRegister(hnrf, 0x07, 0x70);
}
