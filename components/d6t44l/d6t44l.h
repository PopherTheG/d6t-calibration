#ifndef COMPONENTS_D6T44L_D6T44L
#define COMPONENTS_D6T44L_D6T44L

#define D6T_I2C_PORT    0
#define D6T_ADDR        0x0A
#define D6T_CMD         0x4C

#define N_ROW           4
#define N_PIXEL         (4 * 4)
#define N_READ          ((N_PIXEL + 1) * 2 + 1)

#define WRITE_ADDR      ((D6T_ADDR << 1) | I2C_MASTER_WRITE)
#define READ_ADDR       ((D6T_ADDR << 1) | I2C_MASTER_READ)
// #define CALIBRATE
#define SAMPLING        10
// #define DEBUG

#define TEMPERATURE_THRESHOLD   37.4

typedef enum {
    TEMP_EVT_DATA_READY,
    TEMP_EVT_ERROR,
    TEMP_EVT_RESET,
    TEMP_EVT_TEMP_FAIL,
    TEMP_EVT_TEMP_PASS,
} d6t44l_event_id_t;

typedef struct d6t44l_event {
    d6t44l_event_id_t id;    
} d6t44l_event_t;

typedef void (*d6t44l_app_cb)(d6t44l_event_t *evt);

void D6T44L_start_sampling(void);

int D6T44L_init(d6t44l_app_cb app_cb);

void D6T44L_app_run(void);

void D6T44L_app_stop(void);

void D6T44L_update_temp(void);

void D6T44l_get_data(double *temp);

void D6T44L_reset(void);

#endif /* COMPONENTS_D6T44L_D6T44L */