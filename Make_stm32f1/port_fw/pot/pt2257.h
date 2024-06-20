#ifndef PT2257_H
#define PT2257_H

#include "stdint.h"
#include "stdbool.h"

typedef struct
{
    int (*i2c_init)(void);
    int (*i2c_tx)(uint8_t *data, uint32_t size);
    int (*i2c_rx)(uint8_t *data, uint32_t size);
    int (*i2c_tx_rx)(uint8_t *tx_data, uint8_t *data, uint32_t size);
} pt2257_drv_t;

int pt2257_init(pt2257_drv_t *drv);

int pt2257_set_vol(pt2257_drv_t *drv, uint8_t dB);
int pt2257_set_vol_left(pt2257_drv_t *drv, uint8_t dB);
int pt2257_set_vol_right(pt2257_drv_t *drv, uint8_t dB);
int pt2257_mute(pt2257_drv_t *drv, bool toggle);
int pt2257_off(pt2257_drv_t *drv);

#endif /* PT2257_H */

