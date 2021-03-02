#ifndef COMPONENTS_VL53L3CX_VL53L3CX
#define COMPONENTS_VL53L3CX_VL53L3CX

#include <stdint.h>
#include <stdlib.h>

#include "vl53lx_error_codes.h"

#define DISTANCE_THRESHOLD  500

typedef enum {
    TOF_EVT_THRESHOLD_INSIDE,
    TOF_EVT_THRESHOLD_OUTSIDE,
} vl53l3cx_event_id_t;

typedef struct vl53l3cx_event {
    vl53l3cx_event_id_t id;    
} vl53l3cx_event_t;

typedef void (*vl53l3cx_app_cb)(vl53l3cx_event_t *evt);

void vl53l3cx_reset(void);

VL53LX_Error init_vl53l3cx(vl53l3cx_app_cb app_cb);

void vl53l3cx_start_app(void);

void vl53l3cx_stop_app(void);

#endif /* COMPONENTS_VL53L3CX_VL53L3CX */
