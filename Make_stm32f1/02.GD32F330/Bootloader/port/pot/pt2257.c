/**
 * Evc_pt2257.h - Library for using PT2257 - Electronic Volume Controller IC.
 * December 9, 2014.
 * @author Victor NPB
 * @see https://github.com/victornpb/Evc_pt2257/
 */

#include "pt2257.h"

/*

PT2257 - Electronic Volume Controller IC
Datasheet - http://www.princeton.com.tw/Portals/0/Product/PT2257.pdf

Pinout
   |-----_-----|
1 -| Lin   Rin |- 8
2 -| Lout Rout |- 7
3 -| Gnd    V+ |- 6
4 -| SDA   SCL |- 5
   |-----------|

The interface protocol consists of the following:
• A Start bit
• A Chip Address uint8_t=88H 10001000
• ACK=Acknowledge bit
• A Data uint8_t
• A Stop bit

Max. clock speed=100K bits/s

FUNCTION BITS
MSB    2    3    4    5    6    7    LSB  Function
----------------------------------------------------------------------------------------
  1    1    1    1    1    1    1    1    Function OFF (-79dB)
  1    1    0    1    A3   A2   A1   A0   2-Channel, -1dB/step
  1    1    1    0    0    B2   B1   B0   2-Channel, -10dB/step
  1    0    1    0    A3   A2   A1   A0   Left Channel, -1dB/step
  1    0    1    1    0    B2   B1   B0   Left Channel, -10dB/step
  0    0    1    0    A3   A2   A1   A0   Right Channel, -1dB/step
  0    0    1    1    0    B2   B1   B0   Right Channel, -10dB/step
  0    1    1    1    1    0    0    M    2-Channel MUTE (M=1 -> MUTE=ON / M=0 -> MUTE=OFF)

ATTENUATION UNIT BIT  
 A3   AB2  AB1  AB0  ATT(-1) ATT(-10)
  0    0    0    0     0      0  
  0    0    0    1    -1    -10
  0    0    1    0    -2    -20
  0    0    1    1    -3    -30
  0    1    0    0    -4    -40
  0    1    0    1    -5    -50
  0    1    1    0    -6    -60
  0    1    1    1    -7    -70
  1    0    0    0    -8  
  1    0    0    1    -9  
  
*/

//instructions
//#define PT2257_ADDR 0x88        //Chip address
#define EVC_OFF     0xFF  //Function OFF (-79dB)
#define EVC_2CH_1   0xD0  //2-Channel, -1dB/step
#define EVC_2CH_10  0xE0  //2-Channel, -10dB/step
#define EVC_L_1     0xA0  //Left Channel, -1dB/step
#define EVC_L_10    0xB0  //Left Channel, -10dB/step
#define EVC_R_1     0x20  //Right Channel, -1dB/step
#define EVC_R_10    0x30  //Right Channel, -10dB/step
#define EVC_MUTE    0x78  //2-Channel MUTE


/** Constructor */
int pt2257_init(pt2257_drv_t *drv)
{
   return -1; 
}

uint8_t pt2257_level(uint8_t dB)
{
    if (dB>79) dB = 79;
    
    uint8_t b = dB/10;  //get the most significant digit (eg. 79 gets 7)
    uint8_t a = dB%10;  //get the least significant digit (eg. 79 gets 9)
    
    b = b & 0x07; //limit the most significant digit to 3 bit (7)
    
    return (b<<4) | a; //return both numbers in one uint8_t (0BBBAAAA)
}

int pt2257_set_vol(pt2257_drv_t *drv, uint8_t dB)
{
    uint8_t bbbaaaa = pt2257_level(dB);
    
    uint8_t aaaa = bbbaaaa & 0x0F;
    uint8_t bbb = (bbbaaaa>>4) & 0x0F;
    
    bbb = EVC_2CH_10 | bbb;
    aaaa = (EVC_2CH_1 | aaaa);
    
    uint8_t buf[2] = {bbb, aaaa};
    return drv->i2c_tx(buf, 2);
}

int pt2257_set_vol_left(pt2257_drv_t *drv, uint8_t dB)
{
    uint8_t bbbaaaa = pt2257_level(dB);
    
    uint8_t aaaa = bbbaaaa & 0x0F;
    uint8_t bbb = (bbbaaaa>>4) & 0x0F;
    
    uint8_t buf[2] = {EVC_L_10 | bbb, EVC_L_1 | aaaa};
    return drv->i2c_tx(buf, 2);
}

int pt2257_set_vol_right(pt2257_drv_t *drv, uint8_t dB)
{
    uint8_t bbbaaaa = pt2257_level(dB);
    
    uint8_t aaaa = bbbaaaa & 0x0F;
    uint8_t bbb = (bbbaaaa>>4) & 0x0F;
    
    uint8_t buf[2] = {EVC_R_10 | bbb, EVC_R_1 | aaaa};
    return drv->i2c_tx(buf, 2);
}

int pt2257_mute(pt2257_drv_t *drv, bool toggle)
{
    uint8_t buf[1] = {EVC_MUTE | (toggle & 0x01)};
    return drv->i2c_tx(buf, 1);
}

int pt2257_off(pt2257_drv_t *drv)
{    
    uint8_t buf[1] = {EVC_OFF};
    return drv->i2c_tx(buf, 1);
}

