//
// begin license header
//
// This file is part of Pixy CMUcam5 or "Pixy" for short
//
// All Pixy source code is provided under the terms of the
// GNU General Public License v2 (http://www.gnu.org/licenses/gpl-2.0.html).
// Those wishing to use Pixy source code, software and/or
// technologies under different licensing terms should contact us at
// cmucam@cs.cmu.edu. Such licensing terms are available for
// all portions of the Pixy codebase presented here.
//
// end license header
//


#include <stdio.h>
#include "../../../common/inc/pixydefs.h"
#include "../include/spilink.h"
#include "../../../common/inc/debuglog.h"
#include "../include/util.h"

#define PIXY_SPI_CLOCKRATE       2000000

SPILink::SPILink()
{
    m_blockSize = 64;
    m_flags = LINK_FLAG_ERROR_CORRECTED;
}

SPILink::~SPILink()
{
    close();
}

int SPILink::open(uint32_t arg)
{
    SPI.begin();
    SPI.beginTransaction(SPISettings(PIXY_SPI_CLOCKRATE, MSBFIRST, SPI_MODE0));
    //Serial.println("SPI opened");
	return 0;
}

void SPILink::close()
{
    SPI.endTransaction();
}

int SPILink::send(const uint8_t *buf, uint32_t len, uint16_t timeoutMs)
  {
    uint32_t i;
    if (timeoutMs==0){ // 0 equals infinity
        timeoutMs = 100;
    }
    setTimer();

    for (i=0; i<len; i++){
      //Serial.println("Send");
      if(getTimer() >= timeoutMs){
       break;
      }
      SPI.transfer(buf[i]);
    }
    return i;
  }
/*
int SPILink::receive(uint8_t *buf, uint32_t len, uint16_t timeoutMs, uint16_t *cs)
{
  int i;
  if (timeoutMs==0){ // 0 equals infinity
    timeoutMs = 10;
  }
  setTimer();
  if (cs)
    *cs = 0;
  for (i=0; i<len; i++)
  {
    if(getTimer() >= timeoutMs){
        return i;
    }
    buf[i] = SPI.transfer(0x00);
    if (cs)
      *cs += buf[i];
  }
  return i;
}
*/
int SPILink::receive(uint8_t *buf, uint32_t len, uint16_t timeoutMs)
{
  int i;
  if (timeoutMs==0){ // 0 equals infinity
    timeoutMs = 100;
  }
  setTimer();

  for (i=0; i<len; i++)
  {
    if(getTimer() >= timeoutMs){
        break;
    }
    buf[i] = SPI.transfer(0x00);
    //delay(5);
  }
  return i;
}

void SPILink::setTimer()
{
  m_timer = millis();
}

uint32_t SPILink::getTimer()
{
  uint32_t time = millis() - m_timer;

  return time;
}




