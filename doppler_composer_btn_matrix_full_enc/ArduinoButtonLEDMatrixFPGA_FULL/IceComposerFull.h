#ifndef ICECOMPOSERFULL_h
#define ICECOMPOSERFULL_h

#include <ICEClass.h>
#include "/Users/steffensennert/Documents/GitHub/Doppler_FPGA_Firmware/doppler_composer_btn_matrix_full/button_matrix_full.h"



#define COMPOSER_ICE_CDONE      3
#define COMPOSER_ICE_RESET      4
#define COMPOSER_ICE_CS         5


class IceComposerFull : public ICEClass {

  public:
        
  IceComposerFull(){};

        
  void initSPI(){
        ice_cs = COMPOSER_ICE_CS;
        pinMode (COMPOSER_ICE_CS, OUTPUT);
        pinMode (PIN_SPI_SCK,     OUTPUT);
        pinMode (PIN_SPI_MOSI,    OUTPUT);
        pinMode (PIN_SPI_MISO,    INPUT);

        SPIfpga = &SPI;
        SPIfpga->begin();
        pinPeripheral(PIN_SPI_MISO,   PIO_SERCOM_ALT);    // TODO: check in variant may buggy ????
    };

    void reset_inout() {
        //pinMode(ICE_CLK,     INPUT_PULLUP);
        //pinMode(ICE_CDONE,   INPUT_PULLUP);
        //pinMode(ICE_MOSI,    INPUT_PULLUP);
        pinMode(COMPOSER_ICE_RESET,  OUTPUT);
        //pinMode(ICE_CS,      OUTPUT);
        digitalWrite(COMPOSER_ICE_CS, HIGH);
    };


    bool upload(){
        upload(button_matrix_full_bin,sizeof(button_matrix_full_bin));
    }
    
    bool upload(const unsigned char * bitstream , const unsigned int bitstream_size){
        // ensure via are in right mode
        // pinPeripheral(ICE_MISO,   PIO_DIGITAL);
        pinPeripheral(PIN_SPI_MOSI,   PIO_DIGITAL);
        pinPeripheral(PIN_SPI_SCK,    PIO_DIGITAL);
        
        pinMode(PIN_SPI_SCK,     OUTPUT);
        pinMode(PIN_SPI_MOSI,    OUTPUT);
        pinMode(COMPOSER_ICE_RESET,  OUTPUT);
        pinMode(COMPOSER_ICE_CS,      OUTPUT);
        
        // enable reset
        digitalWrite(COMPOSER_ICE_RESET, LOW);
        
        // start clock high
        digitalWrite(PIN_SPI_SCK, HIGH);
        
        // select SRAM programming mode
        digitalWrite(COMPOSER_ICE_CS, LOW);
        delay(10);
        
        // release reset
        digitalWrite(COMPOSER_ICE_RESET, HIGH);
        delay(100);     // TODO: check time! for Waiting FPGA is self init!

        // Begin HardwareSPI
        initSPI();
        SPIfpga->beginTransaction(SPISettings(SPI_FPGA_SPEED, MSBFIRST, SPI_MODE0));
        SPIfpga->transfer(0x00);    // 8 clocks
        //const unsigned int size = sizeof(BITSTREAM);
        for (int k = 0; k < bitstream_size; k++) {
            SPIfpga->transfer(bitstream[k]);
        }
        SPIfpga->endTransaction();
        for(int i = 0 ; i < 8 ; i++){
            SPIfpga->transfer(0x00);
        }
        // End HardwareSPI

        bool cdone_high = digitalRead(COMPOSER_ICE_CDONE) == HIGH;
        reset_inout();
        return cdone_high;
    }



};

#endif // ICECOMPOSERFULL_h




/*
 * ADDED TO ICECLASS.h

     uint32_t sendSPI16_rx32(uint16_t  data) {
        return sendSPI_3Bytes((data >> 16) & 0xff , (data >> 8) & 0xff , data & 0xff );
    }
    
    // send 3 bytes to FPGA
    uint32_t sendSPI_3Bytes(uint8_t  adr , uint8_t  txdata1, uint8_t  txdata2) {
        digitalWrite(ice_cs, LOW);
        SPIfpga->beginTransaction(SPISettings(SPI_FPGA_SPEED, MSBFIRST, SPI_MODE0));
        uint8_t  rxdata1 = SPIfpga->transfer(adr);
        uint8_t  rxdata2 = SPIfpga->transfer(txdata1);
        uint8_t  rxdata3 = SPIfpga->transfer(txdata2);
        digitalWrite(ice_cs, HIGH);
        return rxdata1 << 16 | rxdata2 << 8 | rxdata3 & 0xff ;
    };

 */
