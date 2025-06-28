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

#ifndef _SPILINK_H
#define _SPILINK_H

#include "../../../common/inc/link.h"
#include <SPI.h>


class SPILink : public Link
{
public:
    SPILink();
    virtual ~SPILink();

    int open(uint32_t arg);
    void close();
    virtual int send(const uint8_t *data, uint32_t len, uint16_t timeoutMs);
    virtual int receive(uint8_t *data, uint32_t len, uint16_t timeoutMs);
    //virtual int receive(uint8_t *data, uint32_t len, uint16_t timeoutMs, uint16_t *cs=NULL);
    virtual void setTimer();
    virtual uint32_t getTimer();

private:
    uint32_t m_timer;
};
#endif

