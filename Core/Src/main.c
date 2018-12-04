
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

#include <stdbool.h>
#include <stdlib.h>
#include "vl53l0x_api.h"
//#include "player.h"
//#include "vs10xx.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

uint8_t rgbAddress[] = { 0xC0, 0xC2, 0xC4, 0xC6, 0xC8, 0xCA };

#define RGB_DRIVERS_CNT (sizeof(rgbAddress)/sizeof(rgbAddress[0]))

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void setRGB(size_t index, uint32_t rgb);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, 0x1000);
  return len;
}

//void WriteSci(u_int8 addr, u_int16 data)
//{
//    uint8_t i, tmp = 0b00000010;
//
//    // initially
//    // XCS 1
//    HAL_GPIO_WritePin(VS_XCS_GPIO_Port, VS_XCS_Pin, GPIO_PIN_SET);
//    // SCK 0
//    HAL_GPIO_WritePin(VS_SCK_GPIO_Port, VS_SCK_Pin, GPIO_PIN_RESET);
//    // SI x
//    // SO x
//    // DREQ 1
//    while (HAL_GPIO_ReadPin(VS_DREQ_GPIO_Port, VS_DREQ_Pin) == GPIO_PIN_RESET)
//        HAL_Delay(10);
//
//    // XCS = 0
//    HAL_GPIO_WritePin(VS_XCS_GPIO_Port, VS_XCS_Pin, GPIO_PIN_RESET);
//    // to SI 00000010 with SCK 10
//    for (i = 0; i < 8; i++)
//    {
//        HAL_GPIO_WritePin(VS_SI_GPIO_Port, VS_SI_Pin,
//                ((tmp&0x80)?GPIO_PIN_SET:GPIO_PIN_RESET));
//        tmp = tmp << 1;
//        HAL_GPIO_WritePin(VS_SCK_GPIO_Port, VS_SCK_Pin, GPIO_PIN_SET);
//        HAL_Delay(1);
//        HAL_GPIO_WritePin(VS_SCK_GPIO_Port, VS_SCK_Pin, GPIO_PIN_RESET);
//        HAL_Delay(1);
//    }
//    // to SI msb8address with SCK 10
//    for (i = 0; i < 8; i++)
//    {
//        HAL_GPIO_WritePin(VS_SI_GPIO_Port, VS_SI_Pin,
//                ((addr&0x80)?GPIO_PIN_SET:GPIO_PIN_RESET));
//        addr = addr << 1;
//        HAL_GPIO_WritePin(VS_SCK_GPIO_Port, VS_SCK_Pin, GPIO_PIN_SET);
//        HAL_Delay(1);
//        HAL_GPIO_WritePin(VS_SCK_GPIO_Port, VS_SCK_Pin, GPIO_PIN_RESET);
//        HAL_Delay(1);
//    }
//    // to SI msb16data with SCK 10
//    for (i = 0; i < 16; i++)
//    {
//        HAL_GPIO_WritePin(VS_SI_GPIO_Port, VS_SI_Pin,
//                ((data&0x8000)?GPIO_PIN_SET:GPIO_PIN_RESET));
//        data = data << 1;
//        HAL_GPIO_WritePin(VS_SCK_GPIO_Port, VS_SCK_Pin, GPIO_PIN_SET);
//        HAL_Delay(1);
//        HAL_GPIO_WritePin(VS_SCK_GPIO_Port, VS_SCK_Pin, GPIO_PIN_RESET);
//        HAL_Delay(1);
//    }
//    // DREQ = 0
//    // XCS = 1
//    HAL_GPIO_WritePin(VS_XCS_GPIO_Port, VS_XCS_Pin, GPIO_PIN_SET);
//    // DREQ = 1
//    while (HAL_GPIO_ReadPin(VS_DREQ_GPIO_Port, VS_DREQ_Pin) == GPIO_PIN_RESET)
//        HAL_Delay(10);
//}
//
//u_int16 ReadSci(u_int8 addr)
//{
//    uint8_t i, tmp = 0b00000011;
//    uint16_t data = 0;
//    // initially
//    // XCS 1
//    HAL_GPIO_WritePin(VS_XCS_GPIO_Port, VS_XCS_Pin, GPIO_PIN_SET);
//    // SCK 0
//    HAL_GPIO_WritePin(VS_SCK_GPIO_Port, VS_SCK_Pin, GPIO_PIN_RESET);
//    // SI x
//    // SO x
//    // DREQ 1
//    while (HAL_GPIO_ReadPin(VS_DREQ_GPIO_Port, VS_DREQ_Pin) == GPIO_PIN_RESET)
//        HAL_Delay(10);
//
//    // XCS = 0
//    HAL_GPIO_WritePin(VS_XCS_GPIO_Port, VS_XCS_Pin, GPIO_PIN_RESET);
//    // to SI 00000011 with SCK 10
//    for (i = 0; i < 8; i++)
//    {
//        HAL_GPIO_WritePin(VS_SI_GPIO_Port, VS_SI_Pin,
//                ((tmp&0x80)?GPIO_PIN_SET:GPIO_PIN_RESET));
//        tmp = tmp << 1;
//        HAL_GPIO_WritePin(VS_SCK_GPIO_Port, VS_SCK_Pin, GPIO_PIN_SET);
//        HAL_Delay(1);
//        HAL_GPIO_WritePin(VS_SCK_GPIO_Port, VS_SCK_Pin, GPIO_PIN_RESET);
//        HAL_Delay(1);
//    }
//    // to SI msb8address with SCK 10
//    for (i = 0; i < 8; i++)
//    {
//        HAL_GPIO_WritePin(VS_SI_GPIO_Port, VS_SI_Pin,
//                ((addr&0x80)?GPIO_PIN_SET:GPIO_PIN_RESET));
//        addr = addr << 1;
//        HAL_GPIO_WritePin(VS_SCK_GPIO_Port, VS_SCK_Pin, GPIO_PIN_SET);
//        HAL_Delay(1);
//        HAL_GPIO_WritePin(VS_SCK_GPIO_Port, VS_SCK_Pin, GPIO_PIN_RESET);
//        HAL_Delay(1);
//    }
//    // from SO msb16data with SCK 10
//    for (i = 0; i < 16; i++)
//    {
//        HAL_GPIO_WritePin(VS_SCK_GPIO_Port, VS_SCK_Pin, GPIO_PIN_SET);
//        HAL_Delay(1);
//        if (HAL_GPIO_ReadPin(VS_SO_GPIO_Port, VS_SO_Pin) == GPIO_PIN_SET)
//            data |= (0x0001 << (15-i));
//        HAL_GPIO_WritePin(VS_SCK_GPIO_Port, VS_SCK_Pin, GPIO_PIN_RESET);
//        HAL_Delay(1);
//    }
//    // DREQ = 0
//    // XCS = 1
//    HAL_GPIO_WritePin(VS_XCS_GPIO_Port, VS_XCS_Pin, GPIO_PIN_SET);
//    // DREQ = 1
//    while (HAL_GPIO_ReadPin(VS_DREQ_GPIO_Port, VS_DREQ_Pin) == GPIO_PIN_RESET)
//        HAL_Delay(10);
//
//    return data;
//}

void initRGB(void)
{
    uint8_t addrBuf[4] = { 0x14, 0x15, 0x16, 0x17 };
    uint8_t cfgBuf[4] = { 0b10101010, 0b10101010, 0b10101010, 0b00101010 };
    uint8_t modeBuf[2] = { 0b00000001, 0b00000000 };

    for (size_t i = 0; i < RGB_DRIVERS_CNT; i++)
    {
        HAL_StatusTypeDef res = HAL_I2C_Mem_Write(&hi2c1, rgbAddress[i], 0x00,
                I2C_MEMADD_SIZE_8BIT, modeBuf, 1, 1000);
        if (res != HAL_OK)
        {
            HAL_I2C_DeInit(&hi2c1);
            MX_I2C1_Init();
        }

        HAL_Delay(1);

        for (size_t j = 0; j < 4; j++)
        {
            HAL_StatusTypeDef res = HAL_I2C_Mem_Write(&hi2c1, rgbAddress[i], addrBuf[j], I2C_MEMADD_SIZE_8BIT, &cfgBuf[j], 1, 1000);
            if (res != HAL_OK)
            {
                HAL_I2C_DeInit(&hi2c1);
                MX_I2C1_Init();
            }
        }
    }

    for (size_t i = 0; i < 30; i++)
    {
        setRGB(i, 0);
    }
}

void setRGB(size_t index, uint32_t rgb)
{
    size_t driverIndex = index / 5;
    size_t ledIndex = index % 5;
    if (driverIndex > 1)
        ledIndex = 4 - ledIndex;

    if (driverIndex >= RGB_DRIVERS_CNT) return;

    uint8_t* bytes = (uint8_t*)&rgb;

    HAL_StatusTypeDef res = HAL_I2C_Mem_Write(&hi2c1, rgbAddress[driverIndex],
            (0x02 + ledIndex*3 + 0), I2C_MEMADD_SIZE_8BIT, &bytes[0], 1, 1000);
    if (res != HAL_OK)
    {
        HAL_I2C_DeInit(&hi2c1);
        MX_I2C1_Init();
    }
    res = HAL_I2C_Mem_Write(&hi2c1, rgbAddress[driverIndex],
            (0x02 + ledIndex*3 + 1), I2C_MEMADD_SIZE_8BIT, &bytes[1], 1, 1000);
    if (res != HAL_OK)
    {
        HAL_I2C_DeInit(&hi2c1);
        MX_I2C1_Init();
    }
    res = HAL_I2C_Mem_Write(&hi2c1, rgbAddress[driverIndex],
            (0x02 + ledIndex*3 + 2), I2C_MEMADD_SIZE_8BIT, &bytes[2], 1, 1000);
    if (res != HAL_OK)
    {
        HAL_I2C_DeInit(&hi2c1);
        MX_I2C1_Init();
    }
}

void setRGBAll(uint32_t rgb)
{
    size_t i;
    static uint32_t last = 0;
    if (rgb != last)
    {
        initRGB();
        last = rgb;
        for (i = 0; i < 30; i++)
            setRGB(i, rgb);
    }
}

void print_pal_error(VL53L0X_Error Status){
    char buf[VL53L0X_MAX_STRING_LENGTH];
    VL53L0X_GetPalErrorString(Status, buf);
    printf("API Status: %i : %s\n", Status, buf);
}

VL53L0X_Error WaitMeasurementDataReady(VL53L0X_DEV Dev) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t NewDatReady=0;
    uint32_t LoopNb;

    // Wait until it finished
    // use timeout to avoid deadlock
    if (Status == VL53L0X_ERROR_NONE) {
        LoopNb = 0;
        do {
            Status = VL53L0X_GetMeasurementDataReady(Dev, &NewDatReady);
            if ((NewDatReady == 0x01) || Status != VL53L0X_ERROR_NONE) {
                break;
            }
            LoopNb = LoopNb + 1;
            VL53L0X_PollingDelay(Dev);
        } while (LoopNb < VL53L0X_DEFAULT_MAX_LOOP);

        if (LoopNb >= VL53L0X_DEFAULT_MAX_LOOP) {
            Status = VL53L0X_ERROR_TIME_OUT;
        }
    }

    return Status;
}

VL53L0X_Error WaitStopCompleted(VL53L0X_DEV Dev) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint32_t StopCompleted=0;
    uint32_t LoopNb;

    // Wait until it finished
    // use timeout to avoid deadlock
    if (Status == VL53L0X_ERROR_NONE) {
        LoopNb = 0;
        do {
            Status = VL53L0X_GetStopCompletedStatus(Dev, &StopCompleted);
            if ((StopCompleted == 0x00) || Status != VL53L0X_ERROR_NONE) {
                break;
            }
            LoopNb = LoopNb + 1;
            VL53L0X_PollingDelay(Dev);
        } while (LoopNb < VL53L0X_DEFAULT_MAX_LOOP);

        if (LoopNb >= VL53L0X_DEFAULT_MAX_LOOP) {
            Status = VL53L0X_ERROR_TIME_OUT;
        }

    }

    return Status;
}

uint32_t color = 0;
uint32_t duration = 0;
uint8_t doit = 0;
uint8_t amber = 0;
uint8_t rxbuf[16];

VL53L0X_Error rangingTest(VL53L0X_Dev_t *pMyDevice)
{
    VL53L0X_RangingMeasurementData_t    RangingMeasurementData;
    VL53L0X_RangingMeasurementData_t   *pRangingMeasurementData    = &RangingMeasurementData;
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    uint8_t VhvSettings;
    uint8_t PhaseCal;

    if(Status == VL53L0X_ERROR_NONE)
    {
        printf("Call of VL53L0X_StaticInit\n");
        Status = VL53L0X_StaticInit(pMyDevice); // Device Initialization
        // StaticInit will set interrupt by default
        print_pal_error(Status);
    }

    if(Status == VL53L0X_ERROR_NONE)
    {
        printf("Call of VL53L0X_PerformRefCalibration\n");
        Status = VL53L0X_PerformRefCalibration(pMyDevice,
                &VhvSettings, &PhaseCal); // Device Initialization
        print_pal_error(Status);
    }

    if(Status == VL53L0X_ERROR_NONE)
    {
        printf("Call of VL53L0X_PerformRefSpadManagement\n");
        Status = VL53L0X_PerformRefSpadManagement(pMyDevice,
                &refSpadCount, &isApertureSpads); // Device Initialization
        print_pal_error(Status);
    }

    if(Status == VL53L0X_ERROR_NONE)
    {

        printf("Call of VL53L0X_SetDeviceMode\n");
        Status = VL53L0X_SetDeviceMode(pMyDevice, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING); // Setup in single ranging mode
        print_pal_error(Status);
    }

    if(Status == VL53L0X_ERROR_NONE)
    {
        printf("Call of VL53L0X_StartMeasurement\n");
        Status = VL53L0X_StartMeasurement(pMyDevice);
        print_pal_error(Status);
    }

    if(Status == VL53L0X_ERROR_NONE)
    {
//        uint32_t measurement;
//        uint32_t no_of_measurements = 32;

//        uint16_t* pResults = (uint16_t*)malloc(sizeof(uint16_t) * no_of_measurements);

//        for(measurement=0; measurement<no_of_measurements; measurement++)
        while (Status == VL53L0X_ERROR_NONE)
        {

            Status = WaitMeasurementDataReady(pMyDevice);

            if(Status == VL53L0X_ERROR_NONE)
            {
                Status = VL53L0X_GetRangingMeasurementData(pMyDevice, pRangingMeasurementData);

//                *(pResults + measurement) = pRangingMeasurementData->RangeMilliMeter;
                printf("%d\n", (int)pRangingMeasurementData->RangeMilliMeter);

                if (pRangingMeasurementData->RangeMilliMeter < 600)
                {
                    if (!amber)
                    {
                        setRGBAll(0x0FBF00);
                        amber = 1;
                    }
                }
                else if (amber)
                {
                    setRGBAll(0);
                    amber = 0;
                }

                if (doit)
                {
                    setRGBAll(color);
                    HAL_Delay(duration);
                    if (amber) setRGBAll(0x0FBF00);
                    else setRGBAll(0);
                    doit = 0;
                }

                HAL_UART_Receive_IT(&huart2, rxbuf, 1);

                // Clear the interrupt
                VL53L0X_ClearInterruptMask(pMyDevice, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
                VL53L0X_PollingDelay(pMyDevice);
            } else {
                break;
            }
        }

//        if(Status == VL53L0X_ERROR_NONE)
//        {
//            for(measurement=0; measurement<no_of_measurements; measurement++)
//            {
//                char str[64];
//                snprintf(str, 64, "measurement %d: %d\n", (int)measurement, *(pResults + measurement));
//                uartLog(str);
//            }
//        }

//        free(pResults);
    }


    if(Status == VL53L0X_ERROR_NONE)
    {
        printf("Call of VL53L0X_StopMeasurement\n");
        Status = VL53L0X_StopMeasurement(pMyDevice);
    }

    if(Status == VL53L0X_ERROR_NONE)
    {
        printf("Wait Stop to be competed\n");
        Status = WaitStopCompleted(pMyDevice);
    }

    if(Status == VL53L0X_ERROR_NONE)
    Status = VL53L0X_ClearInterruptMask(pMyDevice,
        VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);

    return Status;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    VL53L0X_Dev_t MyDevice;
    VL53L0X_DeviceInfo_t DeviceInfo;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  initRGB();

//  VSTestInitHardware();
//  VSTestInitSoftware();

//  Mp3Reset();

//  printf("Scanning for I2C devices\n");
//  while (1)
//  {
//      for (uint8_t i = 0; i < 254; i++)
//      {
//          uint8_t data;
//          if (HAL_I2C_Mem_Read(&hi2c1, i, 0x00, I2C_MEMADD_SIZE_8BIT, &data,
//                  1, 50) == HAL_OK)
//          {
//              printf("Found device at address 0x%02X\n", i);
//          }
//          else
//          {
//              HAL_I2C_DeInit(&hi2c1);
//              MX_I2C1_Init();
//          }
//      }
//      HAL_Delay(3000);
//  }

  // Initialize Comms
  MyDevice.I2cDevAddr      = 0x52;
  MyDevice.I2cPtr = &hi2c1;

  if(Status == VL53L0X_ERROR_NONE)
  {
      printf ("Call of VL53L0X_DataInit\n");
      Status = VL53L0X_DataInit(&MyDevice); // Data initialization
      print_pal_error(Status);
  }

  if(Status == VL53L0X_ERROR_NONE)
  {
      Status = VL53L0X_GetDeviceInfo(&MyDevice, &DeviceInfo);
  }
  if(Status == VL53L0X_ERROR_NONE)
  {
      printf("VL53L0X_GetDeviceInfo:\n");
      printf("Device Name : %s\n", DeviceInfo.Name);
      printf("Device Type : %s\n", DeviceInfo.Type);
      printf("Device ID : %s\n", DeviceInfo.ProductId);
      printf("ProductRevisionMajor : %d\n", DeviceInfo.ProductRevisionMajor);
      printf("ProductRevisionMinor : %d\n", DeviceInfo.ProductRevisionMinor);

      if ((DeviceInfo.ProductRevisionMinor != 1) && (DeviceInfo.ProductRevisionMinor != 1)) {
          printf("Error expected cut 1.1 but found cut %d.%d\n",
                  DeviceInfo.ProductRevisionMajor, DeviceInfo.ProductRevisionMinor);
          Status = VL53L0X_ERROR_NOT_SUPPORTED;
      }
  }

  HAL_UART_Receive_IT(&huart2, rxbuf, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//      VsSineTest();

      Status = rangingTest(&MyDevice);
      print_pal_error(Status);

//      for (size_t i = 0; i < 30; i++)
//      {
////          setRGB(i, 0x000F0000);
//          setRGB(i, 0x0000FF00);
//          HAL_Delay(100);
//          setRGB(i, 0);
//      }
//      for (size_t i = 0; i < 30; i++)
//      {
////          setRGB(29-i, 0x000F0000);
//          setRGB(29-i, 0x0000FF00);
//          HAL_Delay(100);
//          setRGB(29-i, 0);
//      }

//      for (size_t i = 0; i < 30; i++)
//          setRGB(i, 128, 0, 0);
//      HAL_Delay(2500);
//      for (size_t i = 0; i < 30; i++)
//          setRGB(i, 0, 128, 0);
//      HAL_Delay(2500);
//      for (size_t i = 0; i < 30; i++)
//          setRGB(i, 0, 0, 12);
//      HAL_Delay(2500);
//      for (size_t i = 0; i < 30; i++)
//          setRGB(i, 0, 0, 0);

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
//      HAL_Delay(2000);
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    static uint8_t msgbuf[32];
    static uint8_t msgind = 0;

    msgbuf[msgind] = rxbuf[0];

    if (msgbuf[msgind] == '\n' || msgbuf[msgind] == '\r')
    {
        msgbuf[msgind] = '\0';

        if (msgind >= 8 && doit == 0)
        {
            msgbuf[6] = '\0';
            sscanf((char*)msgbuf, "%06lx", &color);
            sscanf((char*)&msgbuf[7], "%lu", &duration);

            doit = 1;
        }

        msgind = 0;
    }
    else msgind++;

    if (msgind >= 32) msgind = 0;

    HAL_UART_Receive_IT(&huart2, rxbuf, 1);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
