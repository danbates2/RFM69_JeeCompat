// Native mode RF69 driver.
#include "gpio.h"
#include "spi.h"

#ifndef RF69_SPI_BULK
#define RF69_SPI_BULK 0
#endif

void rfm69_init(uint8_t id, uint8_t group, int freq);
void rfm69_encrypt(const char *key);
void rfm69_txPower(uint8_t level);

int rfm69_receive(void *ptr, int len);
void rfm69_send(uint8_t header, const void *ptr, int len);
void rfm69_sleep();

uint8_t rfm69_rwReg (uint8_t cmd, uint8_t val);
uint8_t rfm69_readReg(uint8_t addr);
void rfm69_writeReg(uint8_t addr, uint8_t val);

int16_t afc;
uint8_t rssi;
uint8_t lna;
uint8_t myId;
uint8_t parity;


uint8_t rfm69_rwReg (uint8_t cmd, uint8_t val) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 0);
    SPI_transfer8(cmd);
    uint8_t in = SPI_transfer8(val);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 1);
    return in;
}


uint8_t rfm69_readReg(uint8_t addr) { return rfm69_rwReg(addr, 0); }
void rfm69_writeReg(uint8_t addr, uint8_t val) { rfm69_rwReg(addr | 0x80, val); }
/*
uint8_t rfm69_readReg(uint8_t addr)
{
  uint8_t regval;
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 0);
  SPI_transfer8(addr & 0x7F);
  regval = SPI_transfer8(0);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 1);
  return regval;
}

void rfm69_writeReg(uint8_t addr, uint8_t value)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 0);
  SPI_transfer8(addr | 0x80);
  SPI_transfer8(value);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 1);
}
*/
#define  REG_FIFO         0x00
#define  REG_OPMODE       0x01
#define  REG_FRFMSB       0x07
#define  REG_PALEVEL      0x11
#define  REG_LNAVALUE     0x18
#define  REG_AFCMSB       0x1F
#define  REG_AFCLSB       0x20
#define  REG_FEIMSB       0x21
#define  REG_FEILSB       0x22
#define  REG_RSSIVALUE    0x24
#define  REG_IRQFLAGS1    0x27
#define  REG_IRQFLAGS2    0x28
#define  REG_SYNCVALUE1   0x2F
#define  REG_SYNCVALUE2   0x30
#define  REG_NODEADDR     0x39
#define  REG_BCASTADDR    0x3A
#define  REG_FIFOTHRESH   0x3C
#define  REG_PKTCONFIG2   0x3D
#define  REG_AESKEYMSB    0x3E

#define  MODE_SLEEP       0 << 2
#define  MODE_STANDBY     1 << 2
#define  MODE_TRANSMIT    3 << 2
#define  MODE_RECEIVE     4 << 2

#define  START_TX    0xC2
#define  STOP_TX     0x42

#define  RCCALSTART          0x80
#define  IRQ1_MODEREADY      1 << 7
#define  IRQ1_RXREADY        1 << 6
#define  IRQ1_SYNADDRMATCH   1 << 0

#define  IRQ2_FIFONOTEMPTY   1 << 6
#define  IRQ2_PACKETSENT     1 << 3
#define  IRQ2_PAYLOADREADY   1 << 2



void rfm69_setMode(uint8_t newMode);
void rfm69_configure(const uint8_t *p);
void rfm69_setFrequency(uint32_t freq);


uint8_t mode;

// driver implementation

void rfm69_setMode(uint8_t newMode) {
  mode = newMode;
  rfm69_writeReg(REG_OPMODE, (rfm69_readReg(REG_OPMODE) & 0xE3) | newMode);
  while ((rfm69_readReg(REG_IRQFLAGS1) & IRQ1_MODEREADY) == 0);
}

void rfm69_setFrequency(uint32_t hz) {
  // accept any frequency scale as input, including KHz and MHz
  // multiply by 10 until freq >= 100 MHz (don't specify 0 as input!)
  while (hz < 100000000)
    hz *= 10;

  // Frequency steps are in units of (32,000,000 >> 19) = 61.03515625 Hz
  // use multiples of 64 to avoid multi-precision arithmetic, i.e. 3906.25 Hz
  // due to this, the lower 6 bits of the calculated factor will always be 0
  // this is still 4 ppm, i.e. well below the radio's 32 MHz crystal accuracy
  // 868.0 MHz = 0xD90000, 868.3 MHz = 0xD91300, 915.0 MHz = 0xE4C000
  uint32_t frf = (hz << 2) / (32000000L >> 11);
  rfm69_writeReg(REG_FRFMSB, frf >> 10);
  rfm69_writeReg(REG_FRFMSB + 1, frf >> 2);
  rfm69_writeReg(REG_FRFMSB + 2, frf << 6);
}

void rfm69_configure(const uint8_t *p) {
  while (true) {
    uint8_t cmd = p[0];
    if (cmd == 0)
      break;
    rfm69_writeReg(cmd, p[1]);
    p += 2;
  }
}

static const uint8_t configRegs[] = {
    // POR value is better for first rf_sleep  0x01, 0x00, // OpMode = sleep
    0x02, 0x00, // DataModul = packet mode, fsk
    0x03, 0x02, // BitRateMsb, data rate = 49,261 khz
    0x04, 0x8A, // BitRateLsb, divider = 32 MHz / 650
    0x05, 0x02, // FdevMsb = 45 KHz
    0x06, 0xE1, // FdevLsb = 45 KHz
    0x0B, 0x20, // Low M
    0x19, 0x4A, // RxBw 100 KHz
    0x1A, 0x42, // AfcBw 125 KHz
    0x1E, 0x0C, // AfcAutoclearOn, AfcAutoOn
    0x25, 0x40, //0x80, // DioMapping1 = SyncAddress (Rx)
    0x26, 0x07, // disable clkout
    0x29, 0xA0, // RssiThresh -80 dB
    0x2D, 0x05, // PreambleSize = 5
    0x2E, 0x88, // SyncConfig = sync on, sync size = 2
    0x2F, 0x2D, // SyncValue1 = 0x2D
    0x37, 0xD0, // PacketConfig1 = fixed, white, no filtering
    0x38, 0x42, // PayloadLength = 0, unlimited
    0x3C, 0x8F, // FifoTresh, not empty, level 15
    0x3D, 0x12, // 0x10, // PacketConfig2, interpkt = 1, autorxrestart off
    0x6F, 0x20, // TestDagc ...
    0x71, 0x02, // RegTestAfc
    0};


void rfm69_init(uint8_t id, uint8_t group, int freq) {
  myId = id;

  // b7 = group b7^b5^b3^b1, b6 = group b6^b4^b2^b0
  parity = group ^ (group << 4);
  parity = (parity ^ (parity << 2)) & 0xC0;

  // 10 MHz, i.e. 30 MHz / 3 (or 4 MHz if clock is still at 12 MHz)
  //spi.master(3);
  //do
  //  rfm69_writeReg(REG_SYNCVALUE1, 0x2D);
  //while (rfm69_readReg(REG_SYNCVALUE1) != 0xAA);
//  do
  //  rfm69_writeReg(REG_SYNCVALUE1, 0x55);
  //while (rfm69_readReg(REG_SYNCVALUE1) != 0x55);

  rfm69_configure(configRegs);
  rfm69_configure(configRegs); // TODO why is this needed ???
  rfm69_setFrequency(freq);

  rfm69_writeReg(REG_SYNCVALUE2, group);
}

void rfm69_encrypt(const char *key) {
  uint8_t cfg = rfm69_readReg(REG_PKTCONFIG2) & ~0x01;
  if (key) {
    for (int i = 0; i < 16; ++i) {
      rfm69_writeReg(REG_AESKEYMSB + i, *key);
      if (*key != 0)
        ++key;
    }
    cfg |= 0x01;
  }
  rfm69_writeReg(REG_PKTCONFIG2, cfg);
}

void rfm69_txPower(uint8_t level) {
  rfm69_writeReg(REG_PALEVEL, (rfm69_readReg(REG_PALEVEL) & ~0x1F) | level);
}

void rfm69_sleep() { rfm69_setMode(MODE_SLEEP); }

int rfm69_receive(void *ptr, int len) {
      if (mode != MODE_RECEIVE)
        rfm69_setMode(MODE_RECEIVE);
      else {
        static uint8_t lastFlag;
        if ((rfm69_readReg(REG_IRQFLAGS1) & IRQ1_RXREADY) != lastFlag) {
          lastFlag ^= IRQ1_RXREADY;
          if (lastFlag) { // flag just went from 0 to 1
            rssi = rfm69_readReg(REG_RSSIVALUE);
            lna = (rfm69_readReg(REG_LNAVALUE) >> 3) & 0x7;
    #if RF69_SPI_BULK
            spi.enable();
            spi.transfer(REG_AFCMSB);
            afc = spi.transfer(0) << 8;
            afc |= spi.transfer(0);
            spi.disable();
    #else
            afc = rfm69_readReg(REG_AFCMSB) << 8;
            afc |= rfm69_readReg(REG_AFCLSB);
    #endif
          }
        }

        if (rfm69_readReg(REG_IRQFLAGS2) & IRQ2_PAYLOADREADY) {

    #if RF69_SPI_BULK
          spi.enable();
          spi.transfer(REG_FIFO);
          int count = spi.transfer(0);
          for (int i = 0; i < count; ++i) {
            uint8_t v = spi.transfer(0);
            if (i < len)
              ((uint8_t *)ptr)[i] = v;
          }
          spi.disable();
    #else
          int count = rfm69_readReg(REG_FIFO);
          for (int i = 0; i < count; ++i) {
            uint8_t v = rfm69_readReg(REG_FIFO);
            if (i < len)
              ((uint8_t *)ptr)[i] = v;
          }
    #endif

          // only accept packets intended for us, or broadcasts
          // ... or any packet if we're the special catch-all node
          uint8_t dest = *(uint8_t *)ptr;
          if ((dest & 0xC0) == parity) {
            uint8_t destId = dest & 0x3F;
            if (destId == myId || destId == 0 || myId == 63)
              return count;
          }
        }
      }
      return -1;
}

void rfm69_send(uint8_t header, const void *ptr, int len) {
  rfm69_setMode(MODE_SLEEP);

#if RF69_SPI_BULK
  spi.enable();
  spi.transfer(REG_FIFO | 0x80);
  spi.transfer(len + 2);
  spi.transfer((header & 0x3F) | parity);
  spi.transfer((header & 0xC0) | myId);
  for (int i = 0; i < len; ++i)
    spi.transfer(((const uint8_t *)ptr)[i]);
  spi.disable();
#else
  rfm69_writeReg(REG_FIFO, len + 2);
  rfm69_writeReg(REG_FIFO, (header & 0x3F) | parity);
  rfm69_writeReg(REG_FIFO, (header & 0xC0) | myId);
  for (int i = 0; i < len; ++i)
    rfm69_writeReg(REG_FIFO, ((const uint8_t *)ptr)[i]);
#endif

  rfm69_setMode(MODE_TRANSMIT);
  while ((rfm69_readReg(REG_IRQFLAGS2) & IRQ2_PACKETSENT) == 0)


  rfm69_setMode(MODE_STANDBY);
}
