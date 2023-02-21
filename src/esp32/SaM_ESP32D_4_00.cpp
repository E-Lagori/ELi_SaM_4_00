/*****************************************
 * Library   : SaM_ESP32D_4_00.cpp - Library for Single ADC with mux for E-LAGORi.
 * Programmer: Anish Bekal
 * Comments  : This library is to use with Dual 3.6A motor driver from E-Lagori
 * Versions  :
 * ------ 	---------- 		-------------------------
 * 0.1.0  	2018-09-12		First beta
 *****************************************/

/*
 * Source for MdM_ESP32D_4_00
 *
 * Copyright (C) 2018  Anish Bekal https://www.e-lagori.com/product/
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
 * This file contains the code for ELagori Single ADC library.
 *
 */

#include "SaM_ESP32D_4_00.h"
#include "driver/ledc.h"
#include "driver/periph_ctrl.h"
#include "SPI.h"

void ARDUINO_ISR_ATTR isr(void *arg) {
  /* Function: Inturrupt Service Routine for acquisition of ADC data continously.
     Input  : pointer to object of class SADCM_4_00
     Output : None
     Performs memory aquisition theough SPI and Stores the values in the <SADCM_4_00>.mem
     When memory overflows the <SADCM_4_00>.memovfl flag is enabled which can be reset after copying the data to different memory location.
  */
  static uint32_t cnt = 0;
  SaM_4_00 *s = static_cast<SaM_4_00*>(arg);
  digitalWrite(s->pins.CS,LOW);
  s->mem[cnt % s->memlen] = s->spi->transfer32(0);
  digitalWrite(s->pins.CS,HIGH);
  cnt++;
  s->memovfl =  (cnt % s->memlen)?1:s->memovfl;
}

void SaM_4_00::setMCLK(uint32_t f = 1024000){
   /* Function: Sets parameters in the timer to generate Master Clock
   	  Input: Frequency of master clock in 32 bit Interger format
	  Output: None
	  This is a private function which uses Timer and PWM channels of ESP32 module to generate 50% duty cycle clock at frequency f. 
	  Maximum value of f is 1.024MHz
   */
   this->ledc_timer.duty_resolution = LEDC_TIMER_1_BIT;
   this->ledc_timer.speed_mode = LEDC_HIGH_SPEED_MODE;
   this->ledc_timer.timer_num = this->tim_num;
   this->ledc_timer.freq_hz = f;
   ledc_timer_config(&(this->ledc_timer));

   // Set up GPIO PIN 

   this->channel_config.channel    = this->ch_num;
   this->channel_config.duty       = 1;
   this->channel_config.gpio_num   = this->pins.MCLK;                      // GPIO pin
   this->channel_config.speed_mode = LEDC_HIGH_SPEED_MODE;
   this->channel_config.timer_sel  = this->tim_num;
   ledc_channel_config(&(this->channel_config));
}

void SaM_4_00::disMCLK(){
   /* Function: Disables Master clock
   	  Input: None
	  Output: None
	  Output: None
	  Disables Master clock running on class defined Timer and LED channel
   */

   // Set up GPIO PIN 

   this->channel_config.channel    = this->ch_num;
   this->channel_config.duty       = 0;
   this->channel_config.gpio_num   = this->pins.MCLK;                      // GPIO pin
   this->channel_config.speed_mode = LEDC_HIGH_SPEED_MODE;
   this->channel_config.timer_sel  = this->tim_num;
   ledc_channel_config(&(this->channel_config));
}


SaM_4_00::SaM_4_00(SaM_4_00_Pinconfig p, ledc_timer_t tim_num, ledc_channel_t){
  /* Function: Constructor for SADCM_4_00 class
     Input: p - E-Lagori control pins attached to various functions. 
	 		tim_num - Timer number, defaults to  Timer 0
			ledc_channel_t - LED channel number, defaults to Channel 0
	 Output: None
	 Configures all the ports of the ESP32 processor and sets the timer and LED channels.
  */

  this->pins = p;
  this->tim_num = tim_num;
  this->ch_num = ch_num;

  //pinMode(this->pins.PD_,OUTPUT);
  pinMode(this->pins.DRDY_,INPUT);
  pinMode(this->pins.MCLK,OUTPUT);
  pinMode(this->pins.CS, OUTPUT);
  pinMode(this->pins.S0, OUTPUT);
  pinMode(this->pins.S1, OUTPUT);
  pinMode(this->pins.S2, OUTPUT);
  pinMode(this->pins.S3, OUTPUT);
  
  digitalWrite(this->pins.S0, LOW);
  digitalWrite(this->pins.S1, LOW);
  digitalWrite(this->pins.S2, LOW);
  digitalWrite(this->pins.S3, LOW);
  this->muxstate = 0;
  // Set up timer
   setMCLK(1024000);
  subproc = 0;
}

SaM_4_00::SaM_4_00(SaM_4_00_Pinconfig p, boardconf brd, ledc_timer_t tim_num, ledc_channel_t){
  /* Function: Constructor for SADCM_4_00 class
     Input: p - E-Lagori control pins attached to various functions. 
	 		tim_num - Timer number, defaults to  Timer 0
			ledc_channel_t - LED channel number, defaults to Channel 0
	 Output: None
	 Configures all the ports of the ESP32 processor and sets the timer and LED channels.
  */

  this->pins = p;
  this->tim_num = tim_num;
  this->ch_num = ch_num;
  this->brd = brd;

  //pinMode(this->pins.PD_,OUTPUT);
  pinMode(this->pins.DRDY_,INPUT);
  pinMode(this->pins.MCLK,OUTPUT);
  pinMode(this->pins.CS, OUTPUT);
  // Set up timer
   setMCLK(1024000);
  subproc = 1;
}

uint32_t SaM_4_00::changMCLK(uint32_t Fs){
	disMCLK();
	this->setMCLK((Fs<32000?Fs:32000)*32);
	return (Fs<32000?Fs:32000);
}
void SaM_4_00::attachSPI(SPIClass *spi){
  /* Function: Attaches the preconfigured SPI channel to SADCM_4_00 board
  	 Input: spi - pointer to object of SPIclass
     Output: None
	 The SPI clock frequency cannot exceed 4MHz. 
  */
  this->spi = spi;
}

void SaM_4_00::SetMux(uint8_t m){
  /* Function: Sets the Mux channel A - P 
  	 Input: m - Use directives SADCM_4_00_CH_A - SADCM_4_00_CH_P to set the channel
     Output: None
	 The Select pins are defined as per SADCM_4_00_Pinconfig in the constructor
  */
  if (!subproc){
  	digitalWrite(this->pins.S0,(m&B00000001));
  	digitalWrite(this->pins.S1,(m&B00000010)>>1);
  	digitalWrite(this->pins.S2,(m&B00000100)>>2);
  	digitalWrite(this->pins.S3,(m&B00001000)>>3);	
  }
  else{
  	char *temp;
  	temp = (char*)&(this->brd);
	Serial2.flush();
  	for (int i = 0;i < 3; i++){
    	Serial2.print(temp[i]);
		//delayMicroseconds(50);
  	}
	//Serial.println(m,BIN);
	Serial2.print((char)m);
  }
  return;
}

uint8_t SaM_4_00::getmuxstate(){
	char temprec[4]; 
	if (subproc){
		char *temp;
	  	temp = (char*)&(this->brd);
		Serial2.flush();
	  	for (int i = 0;i < 3; i++)
	    	Serial2.print(temp[i]);
		Serial2.print(0x30);
		unsigned long sttime = micros();
		while(!Serial2.available()){if((micros() - sttime)>1000) return(0xFF);};
		temprec[0] = Serial2.read();
		temprec[1] = Serial2.read();
		temprec[2] = Serial2.read();
		temprec[3] = Serial2.read();
		if ((temprec[0] == this->brd.reserved) && (temprec[1] == this->brd.brdtype) && (temprec[2] == this->brd.brdno))
			this->muxstate = temprec[3] & 0x0F;
  	}
  	return(this->muxstate);
}

bool SaM_4_00::powerdown(){
	uint8_t temprec[3];
	uint8_t tempstat = 0;
	if (subproc){
		char *temp;
	  	temp = (char*)&(this->brd);
		Serial2.flush();
	  	for (int i = 0;i < 3; i++){
	    	Serial2.print(temp[i]);
	  	}
		Serial2.print(0x10);
		
	  	temp = (char*)&(this->brd);
	  	for (int i = 0;i < 3; i++){
	    	Serial2.print(temp[i]);
	  	}
		Serial2.print(0x40);
		unsigned long sttime = micros();
		while(!Serial2.available()){if((micros() - sttime)>1000) return(0xFF);};
		temprec[0] = Serial2.read();
		temprec[1] = Serial2.read();
		temprec[2] = Serial2.read();
		if ((temprec[0] == this->brd.reserved) && (temprec[1] == this->brd.brdtype) && (temprec[2] == this->brd.brdno))
			tempstat = Serial2.read() & 0x0F;
  	}
	return(tempstat);
}
bool SaM_4_00::powerup(){
	uint8_t temprec[3];
	uint8_t tempstat = 0;
	if (subproc){
		char *temp;
	  	temp = (char*)&(this->brd);
		Serial2.flush();
	  	for (int i = 0;i < 3; i++){
	    	Serial2.print(temp[i]);
	  	}
		Serial2.print(0x11);
	  	temp = (char*)&(this->brd);
	  	for (int i = 0;i < 3; i++){
	    	Serial2.print(temp[i]);
	  	}
		Serial2.print(0x40);
		unsigned long sttime = micros();
		while(!Serial2.available()){if((micros() - sttime)>1000) return(0xFF);};
		temprec[0] = Serial2.read();
		temprec[1] = Serial2.read();
		temprec[2] = Serial2.read();
		if ((temprec[0] == this->brd.reserved) && (temprec[1] == this->brd.brdtype) && (temprec[2] == this->brd.brdno))
			tempstat = Serial2.read() & 0x0F;
  	}
	return(tempstat);
}

void SaM_4_00::initcontacq(uint32_t Fs, int32_t *mem, uint32_t memlen){
  /* Function: Intiates continous acquisition mode. 
  	 Input: Fs - Desired Data sampling frequency - Maximum frequency is 32 kHz
	 		mem - Pointer to the memory where data is to be stored
			memlen - Length of the memory
     Output: None
	 Initiates the continous data aquisition mode where the analog signal is sampled at Fs frequency and stored into the memory
	 location pointed by mem. When the number of acquisitions exceed memelen the storage index restarts to 0 and <SADCM_4_00>.memovfl 
	 flag is turned on. This actions are perfomed using an inturrupt service routine and can run in background. The data stored is in 
	 uint32_t format. Refer to datasheet to convert it into volts. 
  */
   periph_module_enable(PERIPH_LEDC_MODULE);

   this->acqstate = 1;
   this->mem = mem;
   this->memlen = memlen;

   //setup interrupt
   attachInterruptArg(this->pins.DRDY_,isr,this,FALLING);
   // Set up timer
   //this->setMCLK((Fs<32000?Fs:32000)*32);
   changMCLK(Fs);
};

void SaM_4_00::termcontacq(){
  /* Function: Terminate continous acquisition 
  	 Input: None
     Output: None
	 Terminates continous acquisition mode and disables the Master clock and interrupt. 
  */
   disMCLK();
   detachInterrupt(this->pins.DRDY_);
};

void SaM_4_00::applycalcorr(float s = 1, float c = 0){
  /* Function: Set the calibration values of ADC 
  	 Input: s - Slope correction
	 	    c - Bias correction
     Output: None
	 Slope and bias correction to be use to correct the ADC values generated with respect to "ideal" values. If you dont know the 
	 correction values set them to s = 1 and c = 0;  These values can also be set and accessed directly by setting <SADCM_4_00>.slope 
	 and <SADCM_4_00>.bias
  */
  this->slope = s;
  this->bias = c;
}
float SaM_4_00::getADCval(){
  /* Function: Single acquisition of ADC value  
  	 Input: None
     Output: Output
	 Aquires a single data from the ADC. SPI has to be attached before acquisition. 
  */
   int32_t data;

   //Serial.println("Inside getADCval");
   this->acqstate = 0;

   digitalWrite(this->pins.CS,HIGH);
   
   unsigned long prevtime = micros();
   while(digitalRead(this->pins.DRDY_) == 1){if ((micros() - prevtime) > 1000) return -1;};
   while(digitalRead(this->pins.DRDY_) == 0){if ((micros() - prevtime) > 1000) return -1;};
   while(digitalRead(this->pins.DRDY_) == 1){if ((micros() - prevtime) > 1000) return -1;};

	digitalWrite(this->pins.CS,LOW);
	data = this->spi->transfer32(0);
	digitalWrite(this->pins.CS,HIGH);
	
	
	data = (data/2 + 536870912);
	return ((data*5.0/1073741824.00) * this->slope + this->bias);
   
   
   //return ((-data*5.0/2147483648+5)/2 * this->slope + this->bias);
   //return(data);
}

bool SaM_4_00::getacqstate(){
  /* Function: Informs about present acquisition state  
  	 Input: None
     Output: bool
	 Returns the present acquition state of the ADC, 1 indicates continous acquisition mode and 0 indicates Single acquisition mode
  */
   return this->acqstate;
}
