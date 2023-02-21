#include <ELi_McM_4_00.h>
#include <ELi_SaM_4_00.h>

#define MEMLEN 10
struct boardconf Sam_sp = {0B10000000, 0B10011011, 0B10100101};
SaM_4_00 module1({CtrlBUS_T, CtrlBUS_AA, CtrlBUS_AB, 0,0,0,0}, Sam_sp);
SPIClass spi(HSPI);

int32_t mem[MEMLEN];
float maxval = pow(2,24);
void setup() {
  // put your setup code here, to run once:
  uint8_t m;
  Serial.begin(2000000);
  Serial2.begin(115200, SERIAL_8N1, AMISO, AMOSI);
  pinMode(MISO, INPUT);
  pinMode(CtrlBUS_W,OUTPUT);
  digitalWrite(CtrlBUS_W, HIGH);
  spi.begin(SCLK, MISO, MOSI,-1);
  spi.setFrequency(4000000); 
  module1.attachSPI(&spi); 
//  module1.Extpowerdown();
  m  = I_A;//2;
  module1.SetMux(m);
  Serial.print(module1.getmuxstate());
  
//  module1.applycalcorr(1.1503,-0.7878);
  module1.applycalcorr(1,0);
//  for (int i = 0; i<MEMLEN; i++)
//    mem[i] = 0;
//  module1.initcontacq(32000, mem,MEMLEN);
}

uint8_t I[16] = {I_A, I_B, I_C, I_D, I_E, I_F, I_G, I_H, I_I, I_J, I_K, I_L, I_M, I_N, I_O, I_P};
void loop() {
  // put your main code here, to run repeatedly:
 float data[16];
 uint8_t mstate[16], nchan = 3;
 for (uint8_t m = 0; m<nchan;m++){
    module1.SetMux(I[m]);
    do{
      mstate[m] = module1.getmuxstate();
    }while(I[m] != mstate[m]);
    data[m] = module1.getADCval();

    Serial.print(I[m]);
    Serial.print("->");
    Serial.print(mstate[m]);
    Serial.print(": ");
    Serial.println(data[m],3);
    delay(1);
 }
 Serial.println();
  delay(1000);
// if (module1.memovfl){
//    for (int i = 0;i<MEMLEN;i++)
//        Serial.println(mem[i]/16777216.0*5.0,DEC);
//    module1.memovfl = 0;
// }
  

// module1.SetMux(I_H);
// Serial.println(module1.getmuxstate(),BIN);
}
