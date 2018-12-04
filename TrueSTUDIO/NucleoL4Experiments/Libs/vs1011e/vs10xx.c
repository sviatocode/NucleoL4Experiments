/** \file vs10xx.c
 * Functions for interfacing with the mp3 player chip.
 * \todo safe rewind
 * \todo VS1003 WMA "wma-bytes-left" variable adjustment at ff/rew
 */

#include "vs10xx.h"

////////////////////////////////////////////////////////////////////////////////
#include "gpio.h"

#define Mp3SelectControl()      HAL_GPIO_WritePin(VS_XCS_GPIO_Port,     VS_XCS_Pin,     GPIO_PIN_RESET)
#define Mp3DeselectControl()    HAL_GPIO_WritePin(VS_XCS_GPIO_Port,     VS_XCS_Pin,     GPIO_PIN_SET)
#define Mp3SelectData()         HAL_GPIO_WritePin(VS_BSYNC_GPIO_Port,   VS_BSYNC_Pin,   GPIO_PIN_RESET)
#define Mp3DeselectData()       HAL_GPIO_WritePin(VS_BSYNC_GPIO_Port,   VS_BSYNC_Pin,   GPIO_PIN_SET)
#define Mp3PutInReset()         HAL_GPIO_WritePin(VS_RESET_GPIO_Port,   VS_RESET_Pin,   GPIO_PIN_RESET)
#define Mp3ReleaseFromReset()   HAL_GPIO_WritePin(VS_RESET_GPIO_Port,   VS_RESET_Pin,   GPIO_PIN_SET)
#define LED_ON()                HAL_GPIO_WritePin(LD3_GPIO_Port,        LD3_Pin,        GPIO_PIN_SET)
#define LED_OFF()               HAL_GPIO_WritePin(LD3_GPIO_Port,        LD3_Pin,        GPIO_PIN_RESET)
#define MP3_DREQ (HAL_GPIO_ReadPin(VS_DREQ_GPIO_Port, VS_DREQ_Pin) == GPIO_PIN_SET)
#define Delay(ms) HAL_Delay(ms)

void SPIPutChar(uint8_t ch)
{
    uint8_t i;
    for (i = 0; i < 8; i++)
    {
        HAL_GPIO_WritePin(VS_SI_GPIO_Port, VS_SI_Pin,
                ((ch&0x80)?GPIO_PIN_SET:GPIO_PIN_RESET));
        ch = ch << 1;
        HAL_GPIO_WritePin(VS_SCK_GPIO_Port, VS_SCK_Pin, GPIO_PIN_SET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(VS_SCK_GPIO_Port, VS_SCK_Pin, GPIO_PIN_RESET);
        HAL_Delay(1);
    }
}

uint16_t SPIGetChar(void)
{
    uint16_t data = 0;
    uint8_t i;
    HAL_GPIO_WritePin(VS_SI_GPIO_Port, VS_SI_Pin, GPIO_PIN_SET);
    for (i = 0; i < 8; i++)
    {
        HAL_GPIO_WritePin(VS_SCK_GPIO_Port, VS_SCK_Pin, GPIO_PIN_SET);
        HAL_Delay(1);
        if (HAL_GPIO_ReadPin(VS_SO_GPIO_Port, VS_SO_Pin) == GPIO_PIN_SET)
            data |= (0x0001 << (7-i));
        HAL_GPIO_WritePin(VS_SCK_GPIO_Port, VS_SCK_Pin, GPIO_PIN_RESET);
        HAL_Delay(1);
    }
    return data;
}
////////////////////////////////////////////////////////////////////////////////

void Mp3WriteRegister(unsigned char addressbyte,
        unsigned char highbyte, unsigned char lowbyte)
{
    Mp3SelectControl();
    SPIPutChar(VS_WRITE_COMMAND);
    SPIPutChar(addressbyte);
    SPIPutChar(highbyte);
    SPIPutChar(lowbyte);
    Mp3DeselectControl();
}

/** Read the 16-bit value of a VS10xx register */
unsigned int Mp3ReadRegister (unsigned char addressbyte){
  unsigned int resultvalue = 0;

  Mp3SelectControl();
  SPIPutChar(VS_READ_COMMAND);
  SPIPutChar(addressbyte);
  resultvalue = SPIGetChar() << 8;
  resultvalue |= SPIGetChar();
  Mp3DeselectControl();
  return resultvalue;
}
  

/** Soft Reset of VS10xx (Between songs) */
void Mp3SoftReset(){

  /* Soft Reset of VS10xx */
  Mp3WriteRegister (SPI_MODE, 0x08, 0x04); /* Newmode, Reset, No L1-2 */

  Delay(1); /* One millisecond delay */
  while (!MP3_DREQ) /* wait for startup */
    ;
  
  /* Set clock register, doubler etc. */
#ifdef VS1003
  Mp3WriteRegister(SPI_CLOCKF, 0xa6, 0x96);
#else
  Mp3WriteRegister(SPI_CLOCKF, 156, 204);
#endif
  
  Delay(1); /* One millisecond delay */

 
  /* Send null bytes to data interface */
  Mp3SelectData();
  SPIPutChar(0);
  SPIPutChar(0);
  SPIPutChar(0);
  SPIPutChar(0);
  Mp3DeselectData();

}




/** Reset VS10xx */
void Mp3Reset(){

  Mp3PutInReset();
  Delay(1);

  /* Send dummy SPI byte to initialize SPI */
  SPIPutChar(0xFF);

  /* Un-reset MP3 chip */
  Mp3DeselectControl();
  Mp3DeselectData();
  Mp3ReleaseFromReset();
  Mp3SetVolume(0xff,0xff);
  
  /* Set clock register, doubler etc. */
#ifdef VS1003
  Mp3WriteRegister(SPI_CLOCKF, 0xa6, 0x96); //oli 0xa66c
#else
  Mp3WriteRegister(SPI_CLOCKF, 156, 204);
#endif
  
  /* Wait for DREQ */
  while (!MP3_DREQ)
    ;
  
  /* Slow sample rate for slow analog part startup */
  Mp3WriteRegister(SPI_AUDATA, 0, 10); /* 10 Hz */
  Delay(100);

  /* Switch on the analog parts */
  Mp3SetVolume(0xfe,0xfe);
  Mp3WriteRegister (SPI_AUDATA, 31, 64); /* 8kHz */
  Mp3SetVolume(20,20);
  Mp3SoftReset();

  printf("Init: VS10XX\r");

}


/** VS10xx Sine Test Function - Good getting started example */ 
void VsSineTest(){

  /* Reset MP3 chip */
  Mp3PutInReset();       /* Pull xRESET low -> hardware reset */
  Delay(100);            /* 100 ms delay */

  /* Send dummy SPI byte to initialize SPI of Atmel microcontroller */
  SPIPutChar(0xFF);

  /* Un-reset MP3 chip */
  Mp3DeselectControl();  /* Pull xCS high    */
  Mp3DeselectData();     /* Pull xDCS high   */
  Mp3ReleaseFromReset(); /* Pull xRESET high */
  Delay(100);            /* 100 ms delay     */

  LED_ON();

  /* VS10xx Application Notes, chapter 4.8 ---------------------------------*/
  /* As an example, let's write value 0x0820 to register 00 byte by byte    */
  Mp3SelectControl();    /* Now SPI writes go to SCI port                   */
  SPIPutChar(0x02);      /* Send SPI Byte, then wait for byte to be sent.   */
  SPIPutChar(0x00);      /* 0x02 was WRITE command, 0x00 is register number */
  SPIPutChar(0x08);      /* This byte goes to MSB                           */
  SPIPutChar(0x20);      /* ..and this is LSB. (0x20=Allow Test Mode)       */
  Mp3DeselectControl();  /* Now SPI writes don't go to SCI port             */

  while (!MP3_DREQ)      /* Wait for DREQ = 1                               */
    ; 			 /* Do nothing while waiting for DREQ = 1           */

  /* Send a Sine Test Header to Data port                                   */
  Mp3SelectData();       /* Now SPI writes go to SDI port                   */

  SPIPutChar(0x53);      /* - This is a special VLSI Solution test header - */
  SPIPutChar(0xef);      /* - that starts a sine sound. It's good for     - */
  SPIPutChar(0x6e);      /* - testing your code, the chip and also for    - */
  SPIPutChar(0x44);      /* - seeing if your MP3 decoder was manufactured - */
  SPIPutChar(0x00);      /* - by VLSI Solution oy. ------------------------ */
  SPIPutChar(0x00);
  SPIPutChar(0x00);
  SPIPutChar(0x00);
  Mp3DeselectData();
  
  Delay (500);           /* 500 ms delay */
  LED_OFF();

  /* Stop the sine test sound */
  Mp3SelectData();
  SPIPutChar(0x45);
  SPIPutChar(0x78);
  SPIPutChar(0x69);
  SPIPutChar(0x74);
  SPIPutChar(0x00);
  SPIPutChar(0x00);
  SPIPutChar(0x00);
  SPIPutChar(0x00);
  Mp3DeselectData();

  Delay(500);            /* 500 ms delay */
}  

/** Send 2048 zeros. */
void SendZerosToVS10xx(){
  unsigned long i;
    Mp3SelectData();
  SPIPutChar(0);
  for (i=0; i<1048; i++){ /* TESTING 1048 TESTING */
    while (!MP3_DREQ)
      ;
    SPIPutChar(0);
  }
  Mp3DeselectData();
}  
