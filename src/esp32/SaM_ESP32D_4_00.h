/* Header for SaM_ESP32D_4_00.h
 *
 * Copyright (C) 2022  E-LAGORi https://github.com/E-Lagori/ELi_SaM_4_00/tree/main/src/esp32
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * This header file describes the public API for SerialDebug.
 *
 */
 
 #ifndef SaM_ESP32D_4_00.h

#define SaM_ESP32D_4_00.h

#define SaM_CH_A 00
#define SaM_CH_B 01
#define SaM_CH_C 02
#define SaM_CH_D 03
#define SaM_CH_E 04
#define SaM_CH_F 05
#define SaM_CH_G 06
#define SaM_CH_H 07
#define SaM_CH_I 08
#define SaM_CH_J 09
#define SaM_CH_K 10
#define SaM_CH_L 11
#define SaM_CH_M 12
#define SaM_CH_N 13
#define SaM_CH_O 14
#define SaM_CH_P 15


#include <ELi_McM_4_00.h>
#include "driver/ledc.h"
#include "driver/periph_ctrl.h"
#include "SPI.h"

struct SaM_4_00_Pinconfig{
            uint8_t DRDY_, MCLK, CS, S0, S1, S2, S3;
          };

struct boardconf{
  uint8_t reserved;
  uint8_t brdtype;
  uint8_t brdno;
};

class SaM_4_00{
  private:
          ledc_timer_t tim_num; //Timer Number for Master clock generation
          ledc_channel_t ch_num; //Channel Number for Maseter clock generation
          ledc_timer_config_t ledc_timer;// Timer configuration
          ledc_channel_config_t channel_config; // Channel configuration
		  bool acqstate; //0 - Single acq, 1 - Continous acq
		  bool subproc;
		  boardconf brd;
		  uint8_t muxstate;
          SPIClass *spi; // SPI communication 
          uint32_t memlen; // Memory length for continous acquisition
          int32_t *mem; // Memory for continous acquisition
          float slope = 1, bias = 0; // Slope and Bias for ADC calibration

          void setMCLK(uint32_t); // Set Master clock at specified freqeuncy
          void disMCLK(); // Disable master clock
  
  protected:
          SaM_4_00_Pinconfig pins; // pins attached to the system on E-Lagori boards
  public:
        bool memovfl = 0; //Continous acquisiont memory overflow
        SaM_4_00(SaM_4_00_Pinconfig, ledc_timer_t tim_num = LEDC_TIMER_0, ledc_channel_t = LEDC_CHANNEL_0); // Constructor for Single ADC
		SaM_4_00(SaM_4_00_Pinconfig, boardconf, ledc_timer_t tim_num = LEDC_TIMER_0, ledc_channel_t = LEDC_CHANNEL_0); // Constructor for Single ADC
        void attachSPI(SPIClass *); // Attach SPI to Single ADC
        void SetMux(uint8_t); // Set Analog signal channel 
        void initcontacq(uint32_t, int32_t *, uint32_t); // Initiate Contnous acquisition mode
        void termcontacq(); // Terminate continous acquisition
		uint32_t changMCLK(uint32_t);
        uint8_t getmuxstate();
		boardconf getaddress();
		bool powerdown();
		bool powerup();
        void applycalcorr(float, float); // Apply correction factors for ADC calibration
        
        float getADCval(); // Get single ADC value
        bool getacqstate(); // Present acquisition state - Continous or single value
        friend void isr(void *); // interrupt service routine for continous aquisition
};

#endif
