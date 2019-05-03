#include <Arduino.h>
#include <SPI.h>
#include <wiring_private.h> // pinPeripheral() function

#ifndef _ICECLASS_DOPPLER_M4_
#define _ICECLASS_DOPPLER_M4_

// give subclass chance to modify the BITSTREAM
#ifndef BITSTREAM
#define BITSTREAM doppler_simple_io_bin
// #include "/Users/svenbraun/Documents/GitHub/Doppler_FPGA_Firmware/doppler_simple_io.h"
#include <doppler_simple_io.h>
#endif



#define SPI_FPGA_SPEED 34000000
//#include <ice40_simple_io.h>


class ICEClass {
    
public:
    
    SPIClass * SPIfpga;
    ICEClass(){};
    
    // see Board variant.h
    uint16_t  ice_cs    = ICE_CS;
    uint16_t  ice_mosi  = ICE_MOSI;
    uint16_t  ice_miso  = ICE_MISO;
    uint16_t  ice_clk   = ICE_CLK;
    
    void initSPI(){
        pinMode (ice_cs, OUTPUT);
        pinMode (ice_clk, OUTPUT);
        pinMode (ice_mosi, OUTPUT);
        pinMode (ice_miso, INPUT_PULLUP);
        SPIfpga = new SPIClass (&sercom5,  ice_miso,  ice_clk,  ice_mosi,  SPI_PAD_3_SCK_1,  SERCOM_RX_PAD_0);
        SPIfpga->begin();
        // remux Arduino Pins for Hardware SPI
        pinPeripheral(ice_miso,   PIO_SERCOM_ALT);
        pinPeripheral(ice_mosi,   PIO_SERCOM_ALT);
        pinPeripheral(ice_clk,    PIO_SERCOM_ALT);
    };
    
    /*
     void initSPIalt(){
     // https://learn.adafruit.com/using-atsamd21-sercom-to-add-more-spi-i2c-serial-ports/creating-a-new-spi
     // https://github.com/arduino/ArduinoCore-samd/blob/master/cores/arduino/SERCOM.h
     
     ice_cs = 11;        //  pa19
     ice_mosi = 13;     //  pa21
     ice_miso = 12;     //  pa20
     ice_clk = 14;      //  pa22
     
     pinMode (ice_cs, OUTPUT);
     pinMode (ice_clk, OUTPUT);
     pinMode (ice_mosi, OUTPUT);
     pinMode (ice_miso, INPUT);
     SPIfpga = new SPIClass (&sercom5,  ice_miso,  ice_clk,  ice_mosi,  SPI_PAD_3_SCK_1,  SERCOM_RX_PAD_2);
     SPIfpga->begin();
     // remux Arduino Pins for Hardware SPI
     pinPeripheral(ice_miso,   PIO_SERCOM);
     pinPeripheral(ice_mosi,   PIO_SERCOM);
     pinPeripheral(ice_clk,    PIO_SERCOM_ALT);
     };
     */
    

    uint16_t sendSPI16(uint16_t  data) {
        return sendSPI((data >> 8) & 0xff , data & 0xff );
    }


    // send 2 bytes to FPGA
    uint16_t sendSPI(uint8_t  adr , uint8_t  txdata) {
        digitalWrite(ice_cs, LOW);
        SPIfpga->beginTransaction(SPISettings(SPI_FPGA_SPEED, MSBFIRST, SPI_MODE0));
        uint8_t  rxdata1 = SPIfpga->transfer(adr);
        uint8_t  rxdata2 = SPIfpga->transfer(txdata);
        /* SPIfpga->transfer(adr);  // only for messure / debug
        SPIfpga->transfer(txdata);
        SPIfpga->transfer(adr);
        SPIfpga->transfer(txdata);
        SPIfpga->transfer(adr);
        SPIfpga->transfer(txdata);
        SPIfpga->transfer(adr);
        SPIfpga->transfer(txdata);
        SPIfpga->endTransaction(); */
        digitalWrite(ice_cs, HIGH);
        return rxdata1 << 8 | rxdata2 & 0xff ;
    };
    

    

    uint32_t sendSPI16_rx32(uint16_t  data) {
        return sendSPI_4Bytes( (data >> 16) & 0xff , (data >> 8) & 0xff, data & 0xff  );
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

    /*
    uint32_t sendSPI16_rx32(uint16_t  data) {
        return sendSPI_4Bytes( (data >> 16) & 0xff , (data >> 8) & 0xff, data & 0xff  );
    }
     */
    
    // send 3 bytes to FPGA
    uint32_t sendSPI_4Bytes(uint8_t  adr , uint8_t  txdata1, uint8_t  txdata2) {
        digitalWrite(ice_cs, LOW);
        SPIfpga->beginTransaction(SPISettings(SPI_FPGA_SPEED, MSBFIRST, SPI_MODE0));
        uint8_t  rxdata1 = SPIfpga->transfer(adr);
        uint8_t  rxdata2 = SPIfpga->transfer(txdata1);
        uint8_t  rxdata3 = SPIfpga->transfer(txdata2);
        uint8_t  rxdata4 = SPIfpga->transfer(0x00);
        digitalWrite(ice_cs, HIGH);
        return rxdata4 << 24 |  rxdata1 << 16 | rxdata2 << 8 | rxdata3 & 0xff ;
    };
    
    
    
    /*
     //
     // SoftSPI in BitBang Mode
     //
     
     void initSPI_Soft(){
     
     ice_cs = 9;
     ice_mosi = 11;
     ice_miso = 10;
     ice_clk = 13;
     
     pinPeripheral(ice_miso,   PIO_DIGITAL);
     pinPeripheral(ice_mosi,   PIO_DIGITAL);
     pinPeripheral(ice_clk,    PIO_DIGITAL);
     pinMode (ice_miso, INPUT_PULLUP);
     pinMode (ice_cs, OUTPUT);
     pinMode (ice_clk, OUTPUT);
     pinMode (ice_mosi, OUTPUT);
     };
     
     
     // SoftSPI for Testing
     uint8_t sendSPI_Soft(uint8_t  adr , uint8_t  txdata) {
     digitalWrite(ice_clk, HIGH);
     digitalWrite(ice_cs, LOW);
     if(adr & 0x80) digitalWrite(ice_mosi,1); else  digitalWrite(ice_mosi,0);  iceClock();
     if(adr & 0x40) digitalWrite(ice_mosi,1); else  digitalWrite(ice_mosi,0);  iceClock();
     if(adr & 0x20) digitalWrite(ice_mosi,1); else  digitalWrite(ice_mosi,0);  iceClock();
     if(adr & 0x10) digitalWrite(ice_mosi,1); else  digitalWrite(ice_mosi,0);  iceClock();
     if(adr & 0x8)  digitalWrite(ice_mosi,1); else  digitalWrite(ice_mosi,0);  iceClock();
     if(adr & 0x4)  digitalWrite(ice_mosi,1); else  digitalWrite(ice_mosi,0);  iceClock();
     if(adr & 0x2)  digitalWrite(ice_mosi,1); else  digitalWrite(ice_mosi,0);  iceClock();
     if(adr & 0x1)  digitalWrite(ice_mosi,1); else  digitalWrite(ice_mosi,0);  iceClock();
     
     uint8_t result = 0;
     if(txdata & 0x80) digitalWrite(ice_mosi,1); else  digitalWrite(ice_mosi,0);  result >>= 1;  result |= iceReadClock() << 7;
     if(txdata & 0x40) digitalWrite(ice_mosi,1); else  digitalWrite(ice_mosi,0);  result >>= 1;  result |= iceReadClock() << 7;
     if(txdata & 0x20) digitalWrite(ice_mosi,1); else  digitalWrite(ice_mosi,0);  result >>= 1;  result |= iceReadClock() << 7;
     if(txdata & 0x10) digitalWrite(ice_mosi,1); else  digitalWrite(ice_mosi,0);  result >>= 1;  result |= iceReadClock() << 7;
     if(txdata & 0x8)  digitalWrite(ice_mosi,1); else  digitalWrite(ice_mosi,0);  result >>= 1;  result |= iceReadClock() << 7;
     if(txdata & 0x4)  digitalWrite(ice_mosi,1); else  digitalWrite(ice_mosi,0);  result >>= 1;  result |= iceReadClock() << 7;
     if(txdata & 0x2)  digitalWrite(ice_mosi,1); else  digitalWrite(ice_mosi,0);  result >>= 1;  result |= iceReadClock() << 7;
     if(txdata & 0x1)  digitalWrite(ice_mosi,1); else  digitalWrite(ice_mosi,0);  result >>= 1;  result |= iceReadClock() << 7;
     digitalWrite(ice_cs, HIGH);
     return result;
     };
     
     bool iceReadClock() {
     digitalWrite(ice_clk, LOW);
     bool r = digitalRead(ice_miso);
     digitalWrite(ice_clk, HIGH);
     return r;
     };
     */
    //
    // Here we start the BIT_STREAM Stuff in BitBang Mode
    // TODO: use also hardware spi
    //
    
    void iceClock() {
        digitalWrite(ice_clk, LOW);
        digitalWrite(ice_clk, HIGH);
    };
    
    void reset_inout() {
        pinMode(ICE_CLK,     INPUT_PULLUP);
        pinMode(ICE_CDONE,   INPUT_PULLUP);
        pinMode(ICE_MOSI,    INPUT_PULLUP);
        pinMode(ICE_CRESET,  OUTPUT);
        //pinMode(ICE_CS,      OUTPUT);
        digitalWrite(ICE_CS, HIGH);
    };
    
    bool upload(){
        upload(BITSTREAM,sizeof(BITSTREAM));
    }
    bool upload(const unsigned char * bitstream , const unsigned int bitstream_size){
        // ensure via are in right mode
        pinPeripheral(ICE_MISO,   PIO_DIGITAL);
        pinPeripheral(ICE_MOSI,   PIO_DIGITAL);
        pinPeripheral(ICE_CLK,    PIO_DIGITAL);
        
        pinMode(ICE_CLK,     OUTPUT);
        pinMode(ICE_MOSI,    OUTPUT);
        pinMode(ICE_CRESET,  OUTPUT);
        pinMode(ICE_CS,      OUTPUT);
        
        // enable reset
        digitalWrite(ICE_CRESET, LOW);
        
        // start clock high
        digitalWrite(ICE_CLK, HIGH);
        
        // select SRAM programming mode
        digitalWrite(ICE_CS, LOW);
        delay(10);
        
        // release reset
        digitalWrite(ICE_CRESET, HIGH);
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

        bool cdone_high = digitalRead(ICE_CDONE) == HIGH;
        reset_inout();
        return cdone_high;
    }
    
    int uploadBitBang() {   // old style
        
        // ensure via are in right mode
        pinPeripheral(ICE_MISO,   PIO_DIGITAL);
        pinPeripheral(ICE_MOSI,   PIO_DIGITAL);
        pinPeripheral(ICE_CLK,    PIO_DIGITAL);
        
        pinMode(ICE_CLK,     OUTPUT);
        pinMode(ICE_MOSI,    OUTPUT);
        pinMode(ICE_CRESET,  OUTPUT);
        pinMode(ICE_CS,      OUTPUT);
        
        // enable reset
        digitalWrite(ICE_CRESET, LOW);
        
        // start clock high
        digitalWrite(ICE_CLK, HIGH);
        
        // select SRAM programming mode
        // digitalWrite(LOAD_FROM_FLASH, LOW);
        // digitalWrite(RPI_ICE_SELECT, LOW);
        digitalWrite(ICE_CS, LOW);
        delay(10);
        
        // release reset
        digitalWrite(ICE_CRESET, HIGH);
        delay(100);
        
        for (int i = 0; i < 8; i++) {
            iceClock();
        }
        
        const unsigned int mosiPin = ICE_MOSI;
        const unsigned int size = sizeof(BITSTREAM);
        for (int k = 0; k < size; k++) {
            byte d = BITSTREAM[k];
            
            // Speedup
            if(d==0){
                digitalWrite(mosiPin,0);
                iceClock();
                iceClock();
                iceClock();
                iceClock();
                iceClock();
                iceClock();
                iceClock();
                iceClock();
                continue;
            }
            
            
            // bang 8 bits if not 0
            if(d & 0x80) digitalWrite(mosiPin,1); else  digitalWrite(mosiPin,0);  iceClock();
            if(d & 0x40) digitalWrite(mosiPin,1); else  digitalWrite(mosiPin,0);  iceClock();
            if(d & 0x20) digitalWrite(mosiPin,1); else  digitalWrite(mosiPin,0);  iceClock();
            if(d & 0x10) digitalWrite(mosiPin,1); else  digitalWrite(mosiPin,0);  iceClock();
            if(d & 0x8)  digitalWrite(mosiPin,1); else  digitalWrite(mosiPin,0);  iceClock();
            if(d & 0x4)  digitalWrite(mosiPin,1); else  digitalWrite(mosiPin,0);  iceClock();
            if(d & 0x2)  digitalWrite(mosiPin,1); else  digitalWrite(mosiPin,0);  iceClock();
            if(d & 0x1)  digitalWrite(mosiPin,1); else  digitalWrite(mosiPin,0);  iceClock();
            
        }
        
        for (int i = 0; i < 49; i++) {
            iceClock();
        }
        
        bool cdone_high = digitalRead(ICE_CDONE) == HIGH;
        reset_inout();
        
        if (!cdone_high) return 0;
        
        return 1;
    };
    
};





#endif /* _ICECLASS_DOPPLER_M4_ */
