/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"
#include <stdio.h>
#include "usbd_cdc_if.h"
#include <stdbool.h>
#include <math.h>
#include <stdlib.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define M_PI   3.14159265358979323846 /* pi */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s2;
I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
extern char buffer[100];
extern USBD_HandleTypeDef hUsbDeviceFS;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S2_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
uint8_t i2c1_pisiRegister(uint8_t, uint8_t, uint8_t);
uint8_t i2c1_beriRegister(uint8_t, uint8_t);
void initCS43L22(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t i2c1_pisiRegister(uint8_t naprava, uint8_t reg, uint8_t podatek) {
  naprava <<= 1;
  return HAL_I2C_Mem_Write(&hi2c1, naprava, reg, I2C_MEMADD_SIZE_8BIT, &podatek, 1, 10);
}

uint8_t i2c1_beriRegister(uint8_t naprava, uint8_t reg) {
  uint8_t podatek;
  naprava <<= 1;
  HAL_I2C_Mem_Read(&hi2c1, naprava, reg, I2C_MEMADD_SIZE_8BIT, &podatek, 1, 1);
  return podatek;
}

void initCS43L22() {
  unsigned char regValue;

  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4,GPIO_PIN_SET);  // spustim iz reseta
  HAL_Delay(1);   // Vsaj 500 ns, da pridemo iz reseta (dokumentacija, stran 17)

  // analog gain 0, mute inverted
  i2c1_pisiRegister(0x4A, 0x0D, 0x01); // CODEC_MAP_PLAYBACK_CTRL1


  // begin initialization sequence (p. 32)
  i2c1_pisiRegister(0x4A, 0x00, 0x99);
  i2c1_pisiRegister(0x4A, 0x47, 0x80);


  regValue = i2c1_beriRegister(0x4A, 0x32);
  i2c1_pisiRegister(0x4A, 0x32, regValue|0x80);

  regValue = i2c1_beriRegister(0x4A, 0x32);
  i2c1_pisiRegister(0x4A, 0x32, regValue & 0x7F);


  i2c1_pisiRegister(0x4A, 0x00, 0x00);
  // end of initialization sequence


  // slusalke vkljucene, zvocnik izkljucen,
  i2c1_pisiRegister(0x4A, 0x04, 0xAF); // CODEC_MAP_PWR_CTRL2


  // analogni gain 0.6, jakost na obeh kanali enaka
  i2c1_pisiRegister(0x4A, 0x0D, 0x70); // CODEC_MAP_PLAYBACK_CTRL1


  // avtomatsko doloci vzorcevalni hitrost, ura se deli z 2
  i2c1_pisiRegister(0x4A, 0x05, 0x81); // CODEC_MAP_CLK_CTRL


  // prenos I2S, 16 bitni podatki za vsak kanal
  i2c1_pisiRegister(0x4A, 0x06, 0x07); // CODEC_MAP_IF_CTRL1


  // upocasni povecanje glasnosti
  i2c1_pisiRegister(0x4A, 0x0A, 0x00);


  // limniter ni vkljucen
  i2c1_pisiRegister(0x4A, 0x27, 0x00);


  // ojacanje vhodnih podatkov za oba kanala
  i2c1_pisiRegister(0x4A, 0x1A, 0x0A);
  i2c1_pisiRegister(0x4A, 0x1B, 0x0A);


  // kontrola tonov
  i2c1_pisiRegister(0x4A, 0x1F, 0x0F);


  // cip je vkljucen
  i2c1_pisiRegister(0x4A, 0x02, 0x9E); // CODEC_MAP_PWR_CTRL1
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2S2_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  typedef struct {
    int16_t levi;
    int16_t desni;
  } vzorec_t;

  __HAL_I2S_ENABLE(&hi2s3);
  __HAL_I2C_ENABLE(&hi2c1);

  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);

  vzorec_t tonA1[109]; // 48000/440=109 - 1 perioda 109 clenov
  vzorec_t tonC1[183];
  vzorec_t tonC2[92];  // 48000/523=92  - 1 perioda 92 clenov
  vzorec_t tonD1[163]; // 48000/293=164 - 1 perioda 164 clenov
  vzorec_t tonE1[145];
  vzorec_t tonF1[138];
  vzorec_t tonG1[122];
  vzorec_t tonH1[97];
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  float fvz = 48000;
    int amp = 16384;
    int n;

    // perioda A1
    for (n=0; n < sizeof(tonA1)/sizeof(vzorec_t); n++) {
      int16_t vzorec = (amp*sinf(2*M_PI*440*n/fvz));
      tonA1[n].levi = vzorec;
      tonA1[n].desni = vzorec;
    }

    for (n=0; n<sizeof(tonC1)/sizeof(vzorec_t);n++) {
    	int16_t vzorec=(amp*sinf(2*M_PI*262*n/fvz));
    	tonC1[n].levi=vzorec;
    	tonC1[n].desni=vzorec;
    }
    for (n=0; n<sizeof(tonD1)/sizeof(vzorec_t);n++) {
    	int16_t vzorec=(amp*sinf(2*M_PI*294*n/fvz));
    	tonD1[n].levi=vzorec;
    	tonD1[n].desni=vzorec;
    }
    for (n=0; n<sizeof(tonE1)/sizeof(vzorec_t);n++) {
    	int16_t vzorec=(amp*sinf(2*M_PI*330*n/fvz));
    	tonE1[n].levi=vzorec;
    	tonE1[n].desni=vzorec;
    }
    for (n=0; n<sizeof(tonF1)/sizeof(vzorec_t);n++) {
    	int16_t vzorec=(amp*sinf(2*M_PI*349*n/fvz));
    	tonF1[n].levi=vzorec;
    	tonF1[n].desni=vzorec;
    }
    for (n=0; n<sizeof(tonG1)/sizeof(vzorec_t);n++) {
    	int16_t vzorec=(amp*sinf(2*M_PI*392*n/fvz));
    	tonG1[n].levi=vzorec;
    	tonG1[n].desni=vzorec;
    }
    for (n=0; n<sizeof(tonH1)/sizeof(vzorec_t);n++) {
    	int16_t vzorec=(amp*sinf(2*M_PI*494*n/fvz));
    	tonH1[n].levi=vzorec;
    	tonH1[n].desni=vzorec;
    }

    // perioda C2
    for (n=0; n<sizeof(tonC2)/sizeof(vzorec_t);n++) {
      int16_t vzorec=(amp*sinf(2*M_PI*523*n/fvz));
      tonC2[n].levi=vzorec;
      tonC2[n].desni=vzorec;
    }

  GPIO_PinState state;
  int32_t stevec = 0;
  bool pritisnjen = false;



  while (1)
  {

	  state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
	 	  if (state) {
	 	  		pritisnjen = true;
	 	  	}
	 	  else {
	 		  if (pritisnjen) {
	 			  	 stevec++;

	 			  	if(stevec == 1) {
	 			  		initCS43L22();
	 				      // CS43L22 iz reseta dvignemo z iniclaizacijo....

	 			  			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);

	 			  			for (n=0; n<262/4; n++) {
	 			  				HAL_I2S_Transmit(&hi2s3, (uint16_t*)&tonC1, sizeof(tonC1)/2, 10); // velikost - st. 16 bitnih vrednosti
	 			  			}
	 			  			for (n=0; n<294/4; n++) {
	 			  				HAL_I2S_Transmit(&hi2s3, (uint16_t*)&tonD1, sizeof(tonD1)/2, 10); // velikost - st. 16 bitnih vrednosti
	 			  			}
	 			  			for (n=0; n<330/4; n++) {
	 			  				HAL_I2S_Transmit(&hi2s3, (uint16_t*)&tonE1, sizeof(tonE1)/2, 10); // velikost - st. 16 bitnih vrednosti
	 			  			}
	 			  			for (n=0; n<349/4; n++) {
	 			  				HAL_I2S_Transmit(&hi2s3, (uint16_t*)&tonF1, sizeof(tonF1)/2, 10); // velikost - st. 16 bitnih vrednosti
	 			  			}
	 			  			for (n=0; n<392/2; n++) {
	 			  				HAL_I2S_Transmit(&hi2s3, (uint16_t*)&tonG1, sizeof(tonG1)/2, 10); // velikost - st. 16 bitnih vrednosti
	 			  			}
	 			  			for (n=0; n<392/2; n++) {
	 			  				HAL_I2S_Transmit(&hi2s3, (uint16_t*)&tonG1, sizeof(tonG1)/2, 10); // velikost - st. 16 bitnih vrednosti
	 			  				 			  			}
	 			  			for (n=0; n<440/2; n++) {
	 			  				HAL_I2S_Transmit(&hi2s3, (uint16_t*)&tonA1, sizeof(tonA1)/2, 10); // velikost - st. 16 bitnih vrednosti
	 			  			}
	 			  			for (n=0; n<440/2; n++) {
	 			  				HAL_I2S_Transmit(&hi2s3, (uint16_t*)&tonA1, sizeof(tonA1)/2, 10); // velikost - st. 16 bitnih vrednosti
	 			  			}
	 				      // V eni sekundi imamo 440 period A1
	 			  			for (n=0; n<392; n++) {
	 			  				HAL_I2S_Transmit(&hi2s3, (uint16_t*)&tonG1, sizeof(tonG1)/2, 10);
	 			  			}

	 				      // CS43L22 neprenehoma pricakuje vzorce. Ker bomo nehali, ga damo v reset
	 				      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);  // pustim v reset
	 				      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);

	 			  		pritisnjen = false;
	 			  		stevec = 0;
	 			  	}
	 		  }
	 	  }
	 	 uint8_t dolzina = strlen(buffer);
		 char buffer2[6];
		 int counter = 0;
		 int k = 0;

	 	 if(dolzina != 0) {
	 		initCS43L22();
	 		//CDC_Transmit_FS((uint8_t*)&buffer, strlen(buffer));
	 	 if (buffer[0] == 'A') {
	 		 int amplituda = 0;
	 		for (int i = 0; i<5; i++) {
	 			amplituda = amplituda * 10 + (buffer[i+4] - '0');
	 		}
	 		amp = amplituda;

	 		for (n=0; n < sizeof(tonA1)/sizeof(vzorec_t); n++) {
	 		      int16_t vzorec = (amp*sinf(2*M_PI*440*n/fvz));
	 		      tonA1[n].levi = vzorec;
	 		      tonA1[n].desni = vzorec;
	 		    }

	 		    for (n=0; n<sizeof(tonC1)/sizeof(vzorec_t);n++) {
	 		    	int16_t vzorec=(amp*sinf(2*M_PI*262*n/fvz));
	 		    	tonC1[n].levi=vzorec;
	 		    	tonC1[n].desni=vzorec;
	 		    }
	 		    for (n=0; n<sizeof(tonD1)/sizeof(vzorec_t);n++) {
	 		    	int16_t vzorec=(amp*sinf(2*M_PI*294*n/fvz));
	 		    	tonD1[n].levi=vzorec;
	 		    	tonD1[n].desni=vzorec;
	 		    }
	 		    for (n=0; n<sizeof(tonE1)/sizeof(vzorec_t);n++) {
	 		    	int16_t vzorec=(amp*sinf(2*M_PI*330*n/fvz));
	 		    	tonE1[n].levi=vzorec;
	 		    	tonE1[n].desni=vzorec;
	 		    }
	 		    for (n=0; n<sizeof(tonF1)/sizeof(vzorec_t);n++) {
	 		    	int16_t vzorec=(amp*sinf(2*M_PI*349*n/fvz));
	 		    	tonF1[n].levi=vzorec;
	 		    	tonF1[n].desni=vzorec;
	 		    }
	 		    for (n=0; n<sizeof(tonG1)/sizeof(vzorec_t);n++) {
	 		    	int16_t vzorec=(amp*sinf(2*M_PI*392*n/fvz));
	 		    	tonG1[n].levi=vzorec;
	 		    	tonG1[n].desni=vzorec;
	 		    }
	 		    for (n=0; n<sizeof(tonH1)/sizeof(vzorec_t);n++) {
	 		    	int16_t vzorec=(amp*sinf(2*M_PI*494*n/fvz));
	 		    	tonH1[n].levi=vzorec;
	 		    	tonH1[n].desni=vzorec;
	 		    }

	 		    // perioda C2
	 		    for (n=0; n<sizeof(tonC2)/sizeof(vzorec_t);n++) {
	 		      int16_t vzorec=(amp*sinf(2*M_PI*523*n/fvz));
	 		      tonC2[n].levi=vzorec;
	 		      tonC2[n].desni=vzorec;
	 		    }
	 		    memset(buffer, 0, 100);
	 	 }

	 	 for (int i = 1; i < dolzina; i++) {
	 		buffer2[k] = buffer[i];
	 		counter++;
	 		k++;
	 		//CDC_Transmit_FS((uint8_t*)&buffer[i], 1);
	 		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
	 		if (counter == 6) {
		 		if (!(strcmp(buffer2, "C1:1/8"))) {
		 			for (n=0; n<262/4; n++) {
		 				HAL_I2S_Transmit(&hi2s3, (uint16_t*)&tonC1, sizeof(tonC1)/2, 10); // velikost - st. 16 bitnih vrednosti
		 			}
		 			//CDC_Transmit_FS((uint8_t*)&buffer2, strlen(buffer2));
		 		 }
		 		else if (!(strcmp(buffer2, "C1:1/4"))) {
		 			for (n=0; n<262/2; n++) {
		 				HAL_I2S_Transmit(&hi2s3, (uint16_t*)&tonC1, sizeof(tonC1)/2, 10); // velikost - st. 16 bitnih vrednosti
		 			}
		 		 }
		 		else if (!(strcmp(buffer2, "C1:1/2"))) {
		 			for (n=0; n<262; n++) {
		 				HAL_I2S_Transmit(&hi2s3, (uint16_t*)&tonC1, sizeof(tonC1)/2, 10); // velikost - st. 16 bitnih vrednosti
		 			}
		 		}
		 		else if (!(strcmp(buffer2, "C1:1/1"))) {
		 			for (n=0; n<262*2; n++) {
		 				HAL_I2S_Transmit(&hi2s3, (uint16_t*)&tonC1, sizeof(tonC1)/2, 10); // velikost - st. 16 bitnih vrednosti
		 			}
		 		}

		 		else if (!(strcmp(buffer2, "A1:1/8"))) {
		 			for (n=0; n<440/4; n++) {
		 				HAL_I2S_Transmit(&hi2s3, (uint16_t*)&tonA1, sizeof(tonA1)/2, 10); // velikost - st. 16 bitnih vrednosti
		 			}
		 		}
		 		else if (!(strcmp(buffer2, "A1:1/4"))) {
		 			for (n=0; n<440/2; n++) {
		 				HAL_I2S_Transmit(&hi2s3, (uint16_t*)&tonA1, sizeof(tonA1)/2, 10); // velikost - st. 16 bitnih vrednosti
		 			}
		 		}
		 		else if (!(strcmp(buffer2, "A1:1/2"))) {
		 			for (n=0; n<440; n++) {
		 				HAL_I2S_Transmit(&hi2s3, (uint16_t*)&tonA1, sizeof(tonA1)/2, 10); // velikost - st. 16 bitnih vrednosti
		 			}
		 		}
		 		else if (!(strcmp(buffer2, "A1:1/1"))) {
		 			for (n=0; n<440*2; n++) {
		 				HAL_I2S_Transmit(&hi2s3, (uint16_t*)&tonA1, sizeof(tonA1)/2, 10); // velikost - st. 16 bitnih vrednosti
		 			}
		 		}

		 		else if (!(strcmp(buffer2, "D1:1/8"))) {
		 			for (n=0; n<294/4; n++) {
		 				HAL_I2S_Transmit(&hi2s3, (uint16_t*)&tonD1, sizeof(tonD1)/2, 10); // velikost - st. 16 bitnih vrednosti
		 			}
		 			//CDC_Transmit_FS((uint8_t*)&buffer2, strlen(buffer2));
		 		}
		 		else if (!(strcmp(buffer2, "D1:1/4"))) {
		 			for (n=0; n<294/2; n++) {
		 				HAL_I2S_Transmit(&hi2s3, (uint16_t*)&tonD1, sizeof(tonD1)/2, 10); // velikost - st. 16 bitnih vrednosti
		 			}
		 		}
		 		else if (!(strcmp(buffer2, "D1:1/2"))) {
		 			for (n=0; n<294; n++) {
		 				HAL_I2S_Transmit(&hi2s3, (uint16_t*)&tonD1, sizeof(tonD1)/2, 10); // velikost - st. 16 bitnih vrednosti
		 			}
		 		}
		 		else if (!(strcmp(buffer2, "D1:1/1"))) {
		 			for (n=0; n<294*2; n++) {
		 				HAL_I2S_Transmit(&hi2s3, (uint16_t*)&tonD1, sizeof(tonD1)/2, 10); // velikost - st. 16 bitnih vrednosti
		 			}
		 		}

		 		else if (!(strcmp(buffer2, "E1:1/8"))) {
		 			for (n=0; n<330/4; n++) {
		 				HAL_I2S_Transmit(&hi2s3, (uint16_t*)&tonE1, sizeof(tonE1)/2, 10); // velikost - st. 16 bitnih vrednosti
		 			}
		 		}
		 		else if (!(strcmp(buffer2, "E1:1/4"))) {
		 			for (n=0; n<330/2; n++) {
		 				HAL_I2S_Transmit(&hi2s3, (uint16_t*)&tonE1, sizeof(tonE1)/2, 10); // velikost - st. 16 bitnih vrednosti
		 			}
		 		}
		 		else if (!(strcmp(buffer2, "E1:1/2"))) {
		 			for (n=0; n<330; n++) {
		 				HAL_I2S_Transmit(&hi2s3, (uint16_t*)&tonE1, sizeof(tonE1)/2, 10); // velikost - st. 16 bitnih vrednosti
		 			}
		 		}
		 		else if (!(strcmp(buffer2, "E1:1/1"))) {
		 			for (n=0; n<330*2; n++) {
		 				HAL_I2S_Transmit(&hi2s3, (uint16_t*)&tonE1, sizeof(tonE1)/2, 10); // velikost - st. 16 bitnih vrednosti
		 			}
		 		}

		 		else if (!(strcmp(buffer2, "F1:1/8"))) {
		 			for (n=0; n<349/4; n++) {
		 				HAL_I2S_Transmit(&hi2s3, (uint16_t*)&tonF1, sizeof(tonF1)/2, 10); // velikost - st. 16 bitnih vrednosti
		 			}
		 		}
		 		else if (!(strcmp(buffer2, "F1:1/4"))) {
		 			for (n=0; n<349/2; n++) {
		 				HAL_I2S_Transmit(&hi2s3, (uint16_t*)&tonF1, sizeof(tonF1)/2, 10); // velikost - st. 16 bitnih vrednosti
		 			}
		 		}
		 		else if (!(strcmp(buffer2, "F1:1/2"))) {
		 			for (n=0; n<349; n++) {
		 				HAL_I2S_Transmit(&hi2s3, (uint16_t*)&tonF1, sizeof(tonF1)/2, 10); // velikost - st. 16 bitnih vrednosti
		 			}
		 		}
		 		else if (!(strcmp(buffer2, "F1:1/1"))) {
		 			for (n=0; n<349*2; n++) {
		 				HAL_I2S_Transmit(&hi2s3, (uint16_t*)&tonF1, sizeof(tonF1)/2, 10); // velikost - st. 16 bitnih vrednosti
		 			}
		 		}
		 		//

		 		else if (!(strcmp(buffer2, "G1:1/8"))) {
		 			for (n=0; n<392/4; n++) {
		 				HAL_I2S_Transmit(&hi2s3, (uint16_t*)&tonG1, sizeof(tonG1)/2, 10); // velikost - st. 16 bitnih vrednosti
		 			}
		 		}
		 		else if (!(strcmp(buffer2, "G1:1/4"))) {
		 			for (n=0; n<392/2; n++) {
		 				HAL_I2S_Transmit(&hi2s3, (uint16_t*)&tonG1, sizeof(tonG1)/2, 10); // velikost - st. 16 bitnih vrednosti
		 			}
		 		}
		 		else if (!(strcmp(buffer2, "G1:1/2"))) {
		 			for (n=0; n<392; n++) {
		 				HAL_I2S_Transmit(&hi2s3, (uint16_t*)&tonG1, sizeof(tonG1)/2, 10); // velikost - st. 16 bitnih vrednosti
		 			}
		 		}
		 		else if (!(strcmp(buffer2, "G1:1/1"))) {
		 			for (n=0; n<392*2; n++) {
		 				HAL_I2S_Transmit(&hi2s3, (uint16_t*)&tonG1, sizeof(tonG1)/2, 10); // velikost - st. 16 bitnih vrednosti
		 			}
		 		}
		 		///
		 		else if (!(strcmp(buffer2, "H1:1/8"))) {
		 			for (n=0; n<494/4; n++) {
		 				HAL_I2S_Transmit(&hi2s3, (uint16_t*)&tonH1, sizeof(tonH1)/2, 10); // velikost - st. 16 bitnih vrednosti
		 			}
		 		}
		 		else if (!(strcmp(buffer2, "H1:1/4"))) {
		 			for (n=0; n<494/2; n++) {
		 				HAL_I2S_Transmit(&hi2s3, (uint16_t*)&tonH1, sizeof(tonH1)/2, 10); // velikost - st. 16 bitnih vrednosti
		 			}
		 		}
		 		else if (!(strcmp(buffer2, "H1:1/2"))) {
		 			for (n=0; n<494; n++) {
		 				HAL_I2S_Transmit(&hi2s3, (uint16_t*)&tonH1, sizeof(tonH1)/2, 10); // velikost - st. 16 bitnih vrednosti
		 			}
		 		}
		 		else if (!(strcmp(buffer2, "H1:1/1"))) {
		 			for (n=0; n<494*2; n++) {
		 				HAL_I2S_Transmit(&hi2s3, (uint16_t*)&tonH1, sizeof(tonH1)/2, 10); // velikost - st. 16 bitnih vrednosti
		 			}
		 		}
		 		////

		 		else if (!(strcmp(buffer2, "C2:1/8"))) {
		 			for (n=0; n<523/4; n++) {
		 				HAL_I2S_Transmit(&hi2s3, (uint16_t*)&tonC2, sizeof(tonC2)/2, 10); // velikost - st. 16 bitnih vrednosti
		 			}
		 		}
		 		else if (!(strcmp(buffer2, "C2:1/4"))) {
		 			for (n=0; n<523/2; n++) {
		 				HAL_I2S_Transmit(&hi2s3, (uint16_t*)&tonC2, sizeof(tonC2)/2, 10); // velikost - st. 16 bitnih vrednosti
		 			}
		 		}
		 		else if (!(strcmp(buffer2, "C2:1/2"))) {
		 			for (n=0; n<523; n++) {
		 				HAL_I2S_Transmit(&hi2s3, (uint16_t*)&tonC2, sizeof(tonC2)/2, 10); // velikost - st. 16 bitnih vrednosti
		 			}
		 		}
		 		else if (!(strcmp(buffer2, "C2:1/1"))) {
		 			for (n=0; n<523*2; n++) {
		 				HAL_I2S_Transmit(&hi2s3, (uint16_t*)&tonC2, sizeof(tonC2)/2, 10); // velikost - st. 16 bitnih vrednosti
		 			}
		 		}

		 		i = i+1;
		 		counter = 0;
		 		k = 0;
		 		CDC_Transmit_FS((uint8_t*)&buffer2, strlen(buffer2));
		 		memset(buffer2, 0 , 100);
	 		}
	 		if (i == dolzina-1) {

	 			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
	 			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);

	 			break;
	 		}

	 	 }
	 	memset(buffer, 0, 100);
	 	 }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 200;
  PeriphClkInitStruct.PLLI2S.PLLI2SM = 5;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DATA_Ready_Pin */
  GPIO_InitStruct.Pin = DATA_Ready_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DATA_Ready_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : INT1_Pin INT2_Pin MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = INT1_Pin|INT2_Pin|MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
