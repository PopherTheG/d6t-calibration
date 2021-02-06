#ifndef COMPONENTS_SSD1306_SSD1306
#define COMPONENTS_SSD1306_SSD1306

#include <stddef.h>

#include "ssd1306_fonts.h"
#include "ssd1306_conf.h"

#ifndef SSD1306_I2C_PORT
#define SSD1306_I2C_PORT        0
#endif

#ifndef SSD1306_I2C_ADDR
#define SSD1306_I2C_ADDR        (0x3C << 1)
#endif

#ifndef SSD1306_HEIGHT
#define SSD1306_HEIGHT          64
#endif

// SSD1306 width in pixels
#ifndef SSD1306_WIDTH
#define SSD1306_WIDTH           128
#endif

#ifndef SSD1306_BUFFER_SIZE
#define SSD1306_BUFFER_SIZE   SSD1306_WIDTH * SSD1306_HEIGHT / 8
#endif


#define BYTE_CMD_SINGLE    0x80
#define BYTE_CMD_STREAM    0x00
#define BYTE_DATA_STREAM   0x40

typedef enum {
    Black = 0x00, // Black color, no pixel
    White = 0x01  // Pixel is set. Color depends on OLED
} SSD1306_COLOR;

// Enumeration for screen colors
typedef enum {
    SSD1306_OK = 0x00,
    SSD1306_ERR = 0x01  // Generic error.
} SSD1306_Error_t;

// Struct to store transformations
typedef struct {
    uint16_t CurrentX;
    uint16_t CurrentY;
    uint8_t Inverted;
    uint8_t Initialized;
    uint8_t DisplayOn;
} SSD1306_t;
typedef struct {
    uint8_t x;
    uint8_t y;
} SSD1306_VERTEX;

void ssd1306_init(void);
void ssd1306_fill(SSD1306_COLOR color);
void ssd1306_update_screen(void);
void ssd1306_draw_pixel(uint8_t x, uint8_t y, SSD1306_COLOR color);
char ssd1306_write_char(char ch, FontDef font, SSD1306_COLOR color);
char ssd1306_write_string(char* str, FontDef font, SSD1306_COLOR color);
void ssd1306_set_cursor(uint8_t x, uint8_t y);
void ssd1306_line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SSD1306_COLOR color);
void ssd1306_draw_arc(uint8_t x, uint8_t y, uint8_t radius, uint8_t start_angle, uint16_t sweep, SSD1306_COLOR color);
void ssd1306_draw_circle(uint8_t par_x, uint8_t par_y, uint8_t par_r, SSD1306_COLOR color);
void ssd1306_polyline(const SSD1306_VERTEX *par_vertex, uint16_t par_size, SSD1306_COLOR color);
void ssd1306_draw_rectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SSD1306_COLOR color);

void ssd1306_set_contrast(const uint8_t value);

void ssd1306_set_display_on(const uint8_t on);

uint8_t ssd1306_get_display_on();

void ssd1306_write_command(uint8_t byte);
void ssd1306_write_data(uint8_t *buffer, size_t buff_size);
SSD1306_Error_t ssd1306_fill_buffer(uint8_t *buf, uint32_t len);

#endif /* COMPONENTS_SSD1306_SSD1306 */
