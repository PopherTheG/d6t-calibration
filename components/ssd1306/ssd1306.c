#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "font8x8_basic.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "esp_system.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "freertos/task.h"

#define TAG "ssd1306"

static uint8_t SSD1306_Buffer[SSD1306_BUFFER_SIZE];

static SSD1306_t SSD1306;

SSD1306_Error_t ssd1306_fill_buffer(uint8_t *buf, uint32_t len)
{
    SSD1306_Error_t ret = SSD1306_ERR;
    if (len < SSD1306_BUFFER_SIZE)
    {
        memcpy(SSD1306_Buffer, buf, len);
        ret = SSD1306_OK;
    }
    return ret;
}

void ssd1306_write_command(uint8_t byte)
{
    esp_err_t err;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SSD1306_I2C_ADDR | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(cmd, BYTE_CMD_SINGLE, 1);
    i2c_master_write_byte(cmd, byte, 1);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(SSD1306_I2C_PORT, cmd, 100 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "error here %d", err);        
    }    
}

void ssd1306_write_data(uint8_t *data, size_t len)
{
    esp_err_t err;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SSD1306_I2C_ADDR | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(cmd, BYTE_DATA_STREAM, true);
    i2c_master_write(cmd, data, len, 1);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(SSD1306_I2C_PORT, cmd, 200 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "error here %d", err);
    }
}

void ssd1306_set_display_on(const uint8_t on)
{
    uint8_t val;
    if (on)
    {
        val = 0xAF;
        SSD1306.DisplayOn = 1;
    }
    else
    {
        val = 0xAE;
        SSD1306.DisplayOn = 0;
    }
    ssd1306_write_command(val);
}

uint8_t ssd1306_get_display_on()
{
    return SSD1306.DisplayOn;
}

void ssd1306_set_contrast(const uint8_t value)
{
    const uint8_t kSetContrastContrlReg = 0x81;
    ssd1306_write_command(kSetContrastContrlReg);
    ssd1306_write_command(value);
}

void ssd1306_fill(SSD1306_COLOR color)
{
    uint32_t i;
    for (i = 0; i < sizeof(SSD1306_Buffer); i++)
    {
        SSD1306_Buffer[i] = (color == Black) ? 0x00 : 0xFF;
    }
}

void ssd1306_update_screen(void)
{
    for (uint8_t i = 0; i < SSD1306_HEIGHT / 8; i++)
    {
        ssd1306_write_command(0xB0 + i);
        ssd1306_write_command(0x00);
        ssd1306_write_command(0x10);
        ssd1306_write_data(&SSD1306_Buffer[SSD1306_WIDTH * i], SSD1306_WIDTH);
    }
}

void ssd1306_init(void)
{
    ssd1306_set_display_on(0); //display off

    ssd1306_write_command(0x20); // set memorry addressing mode
    ssd1306_write_command(0x00); // 00 horizontal addressing mode, 01 vertival addressing mode, 10 page addressing mode(reset); 11 Invalid

    ssd1306_write_command(0xB0); // set page start address for page addressing mode 0-7;

#ifdef SSD1306_MIRROR_VERT
    ssd1306_write_command(0xc0); // mirror vertically
#else
    ssd1306_write_command(0xc8); // set com output scan direction
#endif

    ssd1306_write_command(0x00); //---set low column address
    ssd1306_write_command(0x10); //---set high column address

    ssd1306_write_command(0x40); //--set start line address - CHECK

    ssd1306_set_contrast(0xFF);

#ifdef SSD1306_MIRROR_HORIZ
    ssd1306_write_command(0xA0); // Mirror horizontally
#else
    ssd1306_write_command(0xA1); //--set segment re-map 0 to 127 - CHECK
#endif

#ifdef SSD1306_INVERSE_COLOR
    ssd1306_write_command(0xA7); //--set inverse color
#else
    ssd1306_write_command(0xA6); //--set normal color
#endif

// Set multiplex ratio.
#if (SSD1306_HEIGHT == 128)
    // Found in the Luma Python lib for SH1106.
    ssd1306_write_command(0xFF);
#else
    ssd1306_write_command(0xA8); //--set multiplex ratio(1 to 64) - CHECK
#endif

#if (SSD1306_HEIGHT == 32)
    ssd1306_write_command(0x1F); //
#elif (SSD1306_HEIGHT == 64)
    ssd1306_write_command(0x3F); //
#elif (SSD1306_HEIGHT == 128)
    ssd1306_write_command(0x3F); // Seems to work for 128px high displays too.
#else
#error "Only 32, 64, or 128 lines of height are supported!"
#endif
    ssd1306_write_command(0xA4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content

    ssd1306_write_command(0xD3); //-set display offset - CHECK
    ssd1306_write_command(0x00); //-not offset

    ssd1306_write_command(0xD5); //--set display clock divide ratio/oscillator frequency
    ssd1306_write_command(0xF0); //--set divide ratio

    ssd1306_write_command(0xD9); //--set pre-charge period
    ssd1306_write_command(0x22); //

    ssd1306_write_command(0xDA); //--set com pins hardware configuration - CHECK

#if (SSD1306_HEIGHT == 32)
    ssd1306_write_command(0x02);
#elif (SSD1306_HEIGHT == 64)
    ssd1306_write_command(0x12);
#elif (SSD1306_HEIGHT == 128)
    ssd1306_write_command(0x12);
#else
#error "Only 32, 64, or 128 lines of height are supported!"
#endif

    ssd1306_write_command(0xDB); //--set vcomh
    ssd1306_write_command(0x20); //0x20,0.77xVcc

    ssd1306_write_command(0x8D); //--set DC-DC enable
    ssd1306_write_command(0x14); //
    ssd1306_set_display_on(1);   //--turn on SSD1306 panel

    // Clear screen
    ssd1306_fill(Black);

    // Flush buffer to screen
    ssd1306_update_screen();

    // Set default values for screen object
    SSD1306.CurrentX = 0;
    SSD1306.CurrentY = 0;

    SSD1306.Initialized = 1;
}

void ssd1306_draw_circle(uint8_t par_x, uint8_t par_y, uint8_t par_r, SSD1306_COLOR color)
{
    int32_t x = -par_r;
    int32_t y = 0;
    int32_t err = 2 - 2 * par_r;
    int32_t e2;

    if (par_x >= SSD1306_WIDTH || par_y >= SSD1306_HEIGHT)
    {
        return;
    }

    do
    {
        ssd1306_draw_pixel(par_x - x, par_y + y, color);
        ssd1306_draw_pixel(par_x + x, par_y + y, color);
        ssd1306_draw_pixel(par_x + x, par_y - y, color);
        ssd1306_draw_pixel(par_x - x, par_y - y, color);
        e2 = err;
        if (e2 <= y) {
            y++;
            err = err + (y * 2 + 1);
            if(-x == y && e2 <= x) {
                e2 = 0;
            } 
        } 

        if (e2 > x) {
            x++;
            err = err + (x * 2 + 1);
        }
    } while (x <= 0);
}

void ssd1306_draw_rectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SSD1306_COLOR color)
{
    ssd1306_line(x1, y1, x2, y1, color);
    ssd1306_line(x2, y1, x2, y2, color);
    ssd1306_line(x2, y2, x1, y2, color);
    ssd1306_line(x1, y2, x1, y1, color);
}

void ssd1306_draw_pixel(uint8_t x, uint8_t y, SSD1306_COLOR color)
{
    if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT)
    {
        return;
    }

    if (SSD1306.Inverted)
    {
        color = (SSD1306_COLOR)!color;
    }

    if (color == White)
    {
        SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] |= 1 << (y % 8);
    }
    else
    {
        SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y % 8));
    }
}

char ssd1306_write_char(char ch, FontDef Font, SSD1306_COLOR color)
{
    uint32_t i, b, j;

    if (ch < 32 || ch > 126)
    {
        return 0;
    }

    if (SSD1306_WIDTH < (SSD1306.CurrentX + Font.FontWidth) ||
        SSD1306_HEIGHT < (SSD1306.CurrentY + Font.FontHeight))
    {
        return 0;
    }

    for (i = 0; i < Font.FontHeight; i++)
    {
        b = Font.data[(ch - 32) * Font.FontHeight + i];
        for (j = 0; j < Font.FontWidth; j++)
        {
            if ((b << j) & 0x8000)
            {
                ssd1306_draw_pixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (SSD1306_COLOR)color);
            }
            else
            {
                ssd1306_draw_pixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (SSD1306_COLOR)!color);
            }
        }
    }

    // The current space is now taken
    SSD1306.CurrentX += Font.FontWidth;

    // Return written char for validation
    return ch;
}

char ssd1306_write_string(char *str, FontDef Font, SSD1306_COLOR color)
{
    while (*str)
    {
        if (ssd1306_write_char(*str, Font, color) != *str)
        {
            return *str;
        }

        str++;
    }

    return *str;
}

void ssd1306_set_cursor(uint8_t x, uint8_t y)
{
    SSD1306.CurrentX = x;
    SSD1306.CurrentY = y;
}

void ssd1306_line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SSD1306_COLOR color)
{
    int32_t deltaX = abs(x2 - x1);
    int32_t deltaY = abs(y2 - y1);
    int32_t signX = ((x1 < x2) ? 1 : -1);
    int32_t signY = ((y1 < y2) ? 1 : -1);
    int32_t error = deltaX - deltaY;
    int32_t error2;

    ssd1306_draw_pixel(x2, y2, color);
    while ((x1 != x2) || (y1 != y2))
    {
        ssd1306_draw_pixel(x1, y1, color);
        error2 = error * 2;
        if (error2 > -deltaY)
        {
            error -= deltaY;
            x1 += signX;
        }

        if (error2 < deltaX)
        {
            error += deltaX;
            y1 += signY;
        }
    }
}