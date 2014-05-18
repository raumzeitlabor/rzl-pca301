// @dir pca301.h (2013-08-10)
// RF12 communication library for PCA 301)
//
// authors: ohweh + trilu
// see http://forum.fhem.de/index.php?t=msg&th=11648
//
// This code is derived from RF12.cpp + Ports.cpp being part of JeeLib
// https://github.com/jcw/jeelib
// 2009-05-06 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php
//

#ifndef _PCA301_h
  #define _PCA301_h
  #if defined(ARDUINO) && ARDUINO >= 100
    #include "Arduino.h"
  #else
    #include "WProgram.h"
  #endif
#endif

//- Shorthand for first RFM12B data byte in rf12_buf. ---------------------------------------------------------------------
#define rf12_data       (rf12_buf)

#define RF12_MAXDATA     66             // maximum message size in bytes
#define RF12_868MHZ      2              // 868 MHz frequency band

#define RF12_SLEEP       0              // enter sleep mode
#define RF12_WAKEUP     -1              // wake up from sleep mode

//- PCA301 device settings ------------------------------------------------------------------------------------------------
#define PCA_MAXDEV      20              // max PCA301 devices
#define PCA_MAXRETRIES  5               // how often a device get's polled before considered "dead"

//- struct for EEPROM config read/write -----------------------------------------------------------------------------------
struct struct_pcaDev {
  uint8_t   channel;                    // associated device channel
  uint32_t  devId;                      // device ID
  uint8_t   pState;                     // device powered on/off
  uint16_t  pNow;                       // actual power consumption (W)
  uint16_t  pTtl;                       // total power consumption (KWh)
  uint32_t  nextTX;                     // last packet submitted  
  uint16_t  retries;                    // outstanding answers
};

struct struct_pcaConf {
  uint8_t  numDev;                      // devices in use
  uint16_t pollIntv;                    // polling intervall in 1/10th of seconds for regular devices
  uint16_t deadIntv;                    // retry intervall in 1/10th of seconds for dead devices
  uint8_t  quiet;                       // quiet mode on/off  
  struct struct_pcaDev pcaDev[PCA_MAXDEV];
  uint16_t crc;
};

//- extern struct_pcaDev  pcaDev; -----------------------------------------------------------------------------------------
extern struct_pcaConf pcaConf;

//- Running crc value, should be zero at end. -----------------------------------------------------------------------------
extern volatile uint16_t rf12_crc;

//- Recv/xmit buf including hdr & crc bytes. ------------------------------------------------------------------------------
extern volatile uint8_t rf12_buf[];

//- Call this once to initialize RFM12B and tune it to PCA 301 RF ---------------------------------------------------------
uint8_t rf12_initialize(void);

//- Call this frequently, returns true if a packet has been received. ----------------------------------------------------- 
uint8_t rf12_recvDone(void);

//- Call this to check whether a new transmission can be started. ---------------------------------------------------------
uint8_t rf12_canSend(void);

//- Call this only when rf12_recvDone() or rf12_canSend() return true. ----------------------------------------------------
void rf12_sendStart(void);

//- Call this only when rf12_recvDone() or rf12_canSend() return true. ----------------------------------------------------
void rf12_sendStart(const void* ptr, uint8_t len);

//- This variant loops on rf12_canSend() and then calls rf12_sendStart() asap. --------------------------------------------
void rf12_sendNow(const void* ptr, uint8_t len);

//- Wait for send to finish. ----------------------------------------------------------------------------------------------
void rf12_sendWait(uint8_t mode);

//- Low-level control RFM12B registers. see http://tools.jeelabs.org/rfm12b -----------------------------------------------
uint16_t rf12_control(uint16_t cmd);

//- calculate crc16 for PCA 301 -------------------------------------------------------------------------------------------
uint16_t crc16_pca301_update (uint16_t crc, uint8_t data);

//- load/save/erase config ------------------------------------------------------------------------------------------------
byte loadConf();
void saveConf();
void eraseConf();
void fillConf();


