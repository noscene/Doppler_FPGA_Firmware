#include <Adafruit_NeoPixel_ZeroDMA.h>
#include "IceComposerFull.h"
#include "LedFunctions.h"


#define PIN        A3
#define NUM_PIXELS 252


Adafruit_NeoPixel_ZeroDMA strip(NUM_PIXELS, PIN, NEO_GRB);
IceComposerFull fpga;



void setup() {
  Serial.begin(9600);

  strip.begin();
  strip.setBrightness(15);
  strip.show();

  fpga.upload();
  delay(100);
  fpga.initSPI();  // start SPI runtime Link to FPGA

  Serial.println("\ndebugger");
}


static uint16_t nr=0;
static uint32_t btArray[13];


bool isButtonPressed(uint16_t bt_nr){
  uint16_t i = bt_nr/18;
  uint16_t rest = bt_nr % 18;
  if(i<13)
    return !bitRead(btArray[i],rest);

   return false;
}



void loop() {

    for(int row = 0 ; row < 13 ; row++){
       btArray[row] = fpga.sendSPI16_rx32(~ ( 0x0001 << row)  );
//       Serial.print(btArray[row],HEX);
//       Serial.print(".");
    }
//    Serial.println(" ---");

    updateLEDs();
    delay(12);
}






void updateLEDs(){
    
    // update 12x16 Maatrix
    for (int r = 0; r < 12; r++) {
        int _row = 12 - r;        //  0 = 12- 0
        for (int x = 0; x<16; x++){
            if(!bitRead(btArray[_row],x)) {
                strip.setPixelColor(iForXY(x, r), 0x00ff00);
            } else  {
                strip.setPixelColor(iForXY(x, r), 0x000000); 
            }
        }
    }
    
    // update 1x16 MachineStrip
    for (int x = 0; x<16; x++){
       if(!bitRead(btArray[0],x)) {
            strip.setPixelColor(machineStrip_adr[x], 0x0000ff);
       } else  {
            strip.setPixelColor(machineStrip_adr[x], 0x000000); 
       }
    }
    
    // update 2x12 SceneButtons
    for (int r = 0; r < 12; r++) {
        int _row = 12 - r;        //  0 = 12- 0
        for (int x = 16; x<18; x++){
            if(!bitRead(btArray[_row],x)) {
                strip.setPixelColor(sceneButtons_adr[x-16][r], 0x00ffff);
            } else  {
                strip.setPixelColor(sceneButtons_adr[x-16][r], 0x000000); 
            }
        }
    }

    // update SceneMasterButtons
       if(!bitRead(btArray[0],16)) {
            strip.setPixelColor(sceneMaster1_adr, 0x00ffff);
       } else  {
            strip.setPixelColor(sceneMaster1_adr, 0x000000); 
       }

       if(!bitRead(btArray[0],17)) {
            strip.setPixelColor(sceneMaster2_adr, 0x00ffff);
       } else  {
            strip.setPixelColor(sceneMaster2_adr, 0x000000); 
       }

    strip.show();
}
