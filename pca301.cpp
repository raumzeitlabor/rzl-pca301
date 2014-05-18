// @dir pca301.cpp (2013-08-10)
// RF12 communication library for PCA 301)
//
// authors: ohweh + trilu
// see http://forum.fhem.de/index.php?t=msg&th=11648
//
// This code is derived from RF12.cpp + Ports.cpp being part of JeeLib
// https://github.com/jcw/jeelib
// 2009-05-06 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php
//

#include "pca301.h"
#include <stdint.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/eeprom.h>

// #define OPTIMIZE_SPI 1            // uncomment to write to the RFM12B @ 8 Mhz
#define PINCHG_IRQ 1              // uncomment to use pin-change interrupts - only support on ATmega328

#define RF_MAX   (RF12_MAXDATA + 5)  // maximum transmit / receive buffer: 3 header + data + 2 crc bytes

//  - leave RFM_IRQ set to the INT0 pin, driver code will use attachInterrupt() to hook into that
//  - you can now change RFM_IRQ when enabling PINCHG_IRQ - will use pin change interrupts
//  - please leave SPI_SS, SPI_MOSI, SPI_MISO, and SPI_SCK as is

#define RFM_IRQ           2
#define SS_DDR            DDRB
#define SS_PORT           PORTB
#define SS_BIT            2              // for PORTB: 2 = d.10, 1 = d.9, 0 = d.8

#define SPI_SS            10             // PB2, pin 16
#define SPI_MOSI          11             // PB3, pin 17
#define SPI_MISO          12             // PB4, pin 18
#define SPI_SCK           13             // PB5, pin 19

// RF12 command codes
#define RF_RECEIVER_ON    0x82DD
#define RF_XMITTER_ON     0x823D
#define RF_IDLE_MODE      0x820D
#define RF_SLEEP_MODE     0x8205
#define RF_WAKEUP_MODE    0x8207
#define RF_TXREG_WRITE    0xB800
#define RF_RX_FIFO_READ   0xB000
#define RF_WAKEUP_TIMER   0xE000

// RF12 status bits
#define RF_LBD_BIT        0x0400
#define RF_RSSI_BIT       0x0100

// bits in the node id configuration byte
#define NODE_BAND         0xC0           // frequency band
#define NODE_ACKANY       0x20           // ack on broadcast packets if set
#define NODE_ID           0x1F           // id of this node, as A..Z or 1..31

// transceiver states, these determine what to do with each interrupt
enum {
    TXCRC1, TXCRC2, TXTAIL, TXDONE, TXIDLE,
    TXRECV,
    TXPRE1, TXPRE2, TXPRE3, TXSYN1, TXSYN2,
};

static uint8_t cs_pin = SS_BIT;          // chip select pin

static uint8_t nodeid = 31;              // address of this node - for PCA comm. fixed 31
static uint8_t group  = 212;             // network group - for PCA comm, fixed 212
static volatile uint8_t rxfill;          // number of data bytes in rf12_buf
static volatile int8_t  rxstate;         // current transceiver state

static uint8_t rf12_len = 7; 

#define RETRIES     8                    // stop retrying after 8 times
#define RETRY_MS    1000                 // resend packet every second until ack'ed

volatile uint16_t rf12_crc;              // running crc value
volatile uint8_t  rf12_buf[RF_MAX];      // recv/xmit buf, including hdr & crc bytes

volatile uint16_t eeprom_crc;            // eeprom crc

// function to set chip select pin from within sketch
void rf12_set_cs(uint8_t pin) {        
  if (pin==10) cs_pin = 2; 	            // Dig10, PB2
  if (pin==9)  cs_pin = 1;  	          // Dig9,  PB1
  if (pin==8)  cs_pin = 0;  	          // Dig8,  PB0
}

void rf12_spiInit () {
  bitSet(SS_PORT, cs_pin);
  bitSet(SS_DDR, cs_pin);
  digitalWrite(SPI_SS, 1);
  pinMode(SPI_SS, OUTPUT);
  pinMode(SPI_MOSI, OUTPUT);
  pinMode(SPI_MISO, INPUT);
  pinMode(SPI_SCK, OUTPUT);
  SPCR = _BV(SPE) | _BV(MSTR);
  #if F_CPU > 10000000
    // use clk/2 (2x 1/4th) for sending (and clk/8 for recv, see rf12_xferSlow)
    SPSR |= _BV(SPI2X);
  #endif
  pinMode(RFM_IRQ, INPUT);
  digitalWrite(RFM_IRQ, 1); // pull-up
}

static uint8_t rf12_byte (uint8_t out) {
  SPDR = out;
  // this loop spins 4 usec with a 2 MHz SPI clock
  while (!(SPSR & _BV(SPIF)))
    ;
  return SPDR;
}

static uint16_t rf12_xferSlow (uint16_t cmd) {
  // slow down to under 2.5 MHz
  #if F_CPU > 10000000
    bitSet(SPCR, SPR0);
  #endif
  bitClear(SS_PORT, cs_pin);
  uint16_t reply = rf12_byte(cmd >> 8) << 8;
  reply |= rf12_byte(cmd);
  bitSet(SS_PORT, cs_pin);
  #if F_CPU > 10000000
    bitClear(SPCR, SPR0);
  #endif
  return reply;
}

#if OPTIMIZE_SPI
  static void rf12_xfer (uint16_t cmd) {
    // writing can take place at full speed, even 8 MHz works
    bitClear(SS_PORT, cs_pin);
    rf12_byte(cmd >> 8) << 8;
    rf12_byte(cmd);
    bitSet(SS_PORT, cs_pin);
  }
#else
  #define rf12_xfer rf12_xferSlow
#endif

// This call provides direct access to the RFM12B registers
uint16_t rf12_control(uint16_t cmd) {
  #if PINCHG_IRQ
    #if RFM_IRQ < 8
      bitClear(PCICR, PCIE2);
    #elif RFM_IRQ < 14
      bitClear(PCICR, PCIE0);
    #else
      bitClear(PCICR, PCIE1);
    #endif
  #else
    bitClear(EIMSK, INT0);
  #endif
  uint16_t r = rf12_xferSlow(cmd);
  #if PINCHG_IRQ
    #if RFM_IRQ < 8
      bitSet(PCICR, PCIE2);
    #elif RFM_IRQ < 14
      bitSet(PCICR, PCIE0);
    #else
      bitSet(PCICR, PCIE1);
    #endif
  #else
    bitSet(EIMSK, INT0);
  #endif
  return r;
}

static void rf12_interrupt() {
  // a transfer of 2x 16 bits @ 2 MHz over SPI takes 2x 8 us inside this ISR
  // correction: now takes 2 + 8 µs, since sending can be done at 8 MHz
  rf12_xfer(0x0000);
    
  if (rxstate == TXRECV) {
    uint8_t in = rf12_xferSlow(RF_RX_FIFO_READ);

    rf12_buf[rxfill++] = in;

    rf12_crc = crc16_pca301_update(rf12_crc, in);

    if (rxfill >= rf12_len + 5 || rxfill >= RF_MAX)
      rf12_xfer(RF_IDLE_MODE);
  } else {
    uint8_t out;

    if (rxstate < 0) {
      // uint8_t pos = 3 + rxstate++;
      uint8_t pos = 10 + rxstate++;
      out         = rf12_buf[pos];
      rf12_crc    = crc16_pca301_update(rf12_crc, out);
    } else
      switch (rxstate++) {
        case TXSYN1: out = 0x2D; break;
        // case TXSYN2: out = group; rxstate = - (2 + rf12_len); break;
        case TXSYN2: out = group; rxstate = - (3 + rf12_len); break;
        case TXCRC1: out = rf12_crc >> 8; break;
        case TXCRC2: out = rf12_crc; break;
        case TXDONE: rf12_xfer(RF_IDLE_MODE);   // fall through
        default:     out = 0xAA;
      }
            
    rf12_xfer(RF_TXREG_WRITE + out);
  }
}

#if PINCHG_IRQ
  #if RFM_IRQ < 8
    ISR(PCINT2_vect) {
      while (!bitRead(PIND, RFM_IRQ))
        rf12_interrupt();
    }
  #elif RFM_IRQ < 14
    ISR(PCINT0_vect) { 
      while (!bitRead(PINB, RFM_IRQ - 8))
        rf12_interrupt();
    }
  #else
    ISR(PCINT1_vect) {
      while (!bitRead(PINC, RFM_IRQ - 14))
        rf12_interrupt();
    }
  #endif
#endif

static void rf12_recvStart () {
  rxfill   = 0;
  // rf12_len = 0;
  rf12_crc = 0;
  rxstate  = TXRECV;    
  rf12_xfer(RF_RECEIVER_ON);
}

// returns true if packet was received - modifies rf12_hdr, rf12_len, rf12_data, rf12_crc
uint8_t rf12_recvDone () {
  if (rxstate == TXRECV && (rxfill >= rf12_len + 5 || rxfill >= RF_MAX)) {
    rxstate = TXIDLE;
    if (rf12_len > RF12_MAXDATA)
      rf12_crc = 1;   // force bad crc if packet length is invalid
    return 1;     // it's a broadcast packet or it's addressed to this node
  }
  if (rxstate == TXIDLE)
    rf12_recvStart();
  return 0;
}

// call before sending data - returns true if transmission can be started
uint8_t rf12_canSend () {
  if (rxstate == TXRECV && rxfill == 0 && (rf12_control(0x0000) & RF_RSSI_BIT) == 0) {
    rf12_control(RF_IDLE_MODE); // stop receiver
    rxstate = TXIDLE;
    return 1;
  }
  return 0;
}

void rf12_sendStart (void) {
    rf12_crc = 0;
    rxstate  = TXPRE1;
    rf12_xfer(RF_XMITTER_ON); // bytes will be fed via interrupts
}

void rf12_sendStart (const void* ptr, uint8_t len) {
    // rf12_len = len;
    memcpy((void*) rf12_data, ptr, len);
    rf12_sendStart();
}

// wait until transmission is possible, then start it as soon as possible
void rf12_sendNow (const void* ptr, uint8_t len) {
  while (!rf12_canSend())
    rf12_recvDone(); // keep the driver state machine going, ignore incoming
  rf12_sendStart(ptr, len);
}
  
// wait for completion of rf12_sendStart() using the specified low-power mode
void rf12_sendWait (uint8_t mode) {
  // wait for packet to actually finish sending
  // go into low power mode, as interrupts are going to come in very soon
  while (rxstate != TXIDLE)
    if (mode) {
      // power down mode is only possible if the fuses are set to start
      // up in 258 clock cycles, i.e. approx 4 us - else must use standby!
      // modes 2 and higher may lose a few clock timer ticks
      set_sleep_mode(mode == 3 ? SLEEP_MODE_PWR_DOWN :
          #ifdef SLEEP_MODE_STANDBY
             mode == 2 ? SLEEP_MODE_STANDBY :
          #endif
        SLEEP_MODE_IDLE);
      sleep_mode();
    }
}

// Call this once to initialize RFM12B and tune it to PCA 301 RF
uint8_t rf12_initialize(uint16_t freq) {

  rf12_spiInit();

  rf12_xfer(0x0000);                  // intitial SPI transfer added to avoid power-up problem

  rf12_xfer(RF_SLEEP_MODE);           // DC (disable clk pin), enable lbd

  // wait until RFM12B is out of power-up reset, this takes several *seconds*
  rf12_xfer(RF_TXREG_WRITE);          // in case we're still in OOK mode
  while (digitalRead(RFM_IRQ) == 0)
    rf12_xfer(0x0000);

  // settings derived from SPI trace of "Kleiner" as disccues on
  // http://forum.fhem.de/index.php?t=msg&th=11648
  rf12_xfer(0x80E8);                  // Config       - b868,EL(ena TX),EF(ena RX FIFO),12.5pF
  rf12_xfer(freq);                    // Frequency    - 868.9500MHz
  rf12_xfer(0xC633);                  // DataRate     - 6.631kbps
  rf12_xfer(0x94C5);                  // RX Ctrl      - VDI,FAST,67kHz,0dBm,-73dBm
  rf12_xfer(0xC2AF);                  // Filter/Clock - AL,!ml,DIG,DQD7
  rf12_xfer(0xCA83);                  // FIFO+Reset   - FIFO8,2-SYNC,!ff,DR
  rf12_xfer(0xCE00 | group);          // SYNC         - Sync on 2D XX
  rf12_xfer(0xC477);                  // AFC          - @PWR once,+3..-4,EN,EH,!st,EO
  rf12_xfer(0x9820);                  // TX Ctrl - FS+,45 kHz
  rf12_xfer(0xCC76);                  // PLL Settings 
  rf12_xfer(0xE000);                  // JL/NOT USE 
  rf12_xfer(0xC80E);                  // Low Duty     - EN,D2
  rf12_xfer(0xC049);                  // JL/Low bat   - 1.66MHz,3.1V 

  rxstate = TXIDLE;

  #if PINCHG_IRQ
    #if RFM_IRQ < 8
      if ((nodeid & NODE_ID) != 0) {
        bitClear(DDRD, RFM_IRQ);      // input
        bitSet(PORTD, RFM_IRQ);       // pull-up
        bitSet(PCMSK2, RFM_IRQ);      // pin-change
        bitSet(PCICR, PCIE2);         // enable
      } else
        bitClear(PCMSK2, RFM_IRQ);
    #elif RFM_IRQ < 14
      if ((nodeid & NODE_ID) != 0) {
        bitClear(DDRB, RFM_IRQ - 8);  // input
        bitSet(PORTB, RFM_IRQ - 8);   // pull-up
        bitSet(PCMSK0, RFM_IRQ - 8);  // pin-change
        bitSet(PCICR, PCIE0);         // enable
      } else
        bitClear(PCMSK0, RFM_IRQ - 8);
    #else
      if ((nodeid & NODE_ID) != 0) {
        bitClear(DDRC, RFM_IRQ - 14); // input
        bitSet(PORTC, RFM_IRQ - 14);  // pull-up
        bitSet(PCMSK1, RFM_IRQ - 14); // pin-change
        bitSet(PCICR, PCIE1);         // enable
      } else
        bitClear(PCMSK1, RFM_IRQ - 14);
    #endif
  #else
    if ((nodeid & NODE_ID) != 0)
      attachInterrupt(0, rf12_interrupt, LOW);
    else
      detachInterrupt(0);
  #endif

  return nodeid;
}

uint16_t crc16_pca301_update (uint16_t crc, uint8_t data) {
  int i;
  crc = crc ^ ((uint16_t)data << 8);
  for (i=0; i<8; i++) {
    if (crc & 0x8000)
      crc = (crc << 1) ^ 0x8005;
    else
      crc <<= 1;
  }
  return crc;
}

//- load config from EEPROM - returns 1 if valid config was found, otherwise 0
byte loadConf() {
  uint16_t len   = sizeof(pcaConf);
  byte *pPtrByte = (byte*)&pcaConf;        // byte Ptr to pcaConf
  eeprom_crc     = 0;
  eeprom_read_block(&pcaConf, (void *) 0, len);

  for (int i=0; i < (len - 2); i++) {
    eeprom_crc = crc16_pca301_update(eeprom_crc, *pPtrByte);
    pPtrByte++;
  }

  // valid config in EEPROM?
  if (eeprom_crc == pcaConf.crc) {
    // valid config found, reset dynamic settings
    for (int i = 0; i < pcaConf.numDev; i++) {
      pcaConf.pcaDev[i].pNow    = 0;
      pcaConf.pcaDev[i].pTtl    = 0;
      pcaConf.pcaDev[i].nextTX  = 0;
      pcaConf.pcaDev[i].retries = 0;
    }
    return 1;
  } else {
    // invalid crc
    return 0;
  }
}

// save config to EEPROM
void saveConf() {
  uint16_t len = sizeof(pcaConf);
  byte *pPtrByte = (byte*)&pcaConf;        // byte Ptr to pcaConf

  eeprom_crc = 0;
  for (int i=0; i < (len - 2); i++) {
    eeprom_crc = crc16_pca301_update(eeprom_crc, *pPtrByte);
    pPtrByte++;
  }
  pcaConf.crc = eeprom_crc;
  
  eeprom_write_block(&pcaConf, (void *) 0, len);
}

// erase config
void eraseConf() {
  pcaConf.numDev = 0;
}

//- fill config ------------------------------------------------------------------------------------
void fillConf() {
  pcaConf.numDev     = 0;
  pcaConf.pollIntv   = 300;                             // default poll interval in 1/10th seconds
  pcaConf.deadIntv   = 3000;                            // dead device poll retry interval in 1/10th seconds
  pcaConf.quiet      = 1;                               // quiet, 1=suppress TX and bad packets
  
  pcaConf.pcaDev[0]  = (struct_pcaDev){1 ,0xAAAAA};     // device 1
  pcaConf.pcaDev[1]  = (struct_pcaDev){2 ,0xBBBBB};     // device 2
}


