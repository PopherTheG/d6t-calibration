#include <string.h>
#include <stdio.h>
#include "ssd1306.h"
#include "ssd1306_tests.h"

void ssd1306_TestCircle() {
    uint32_t delta;

    for ( delta = 0; delta < 5; delta++) {
        ssd1306_draw_circle(20*delta+30,  30, 10, White);
    }
    ssd1306_update_screen();
}

void ssd1306_TestLine()
{
    ssd1306_line(1, 1, SSD1306_WIDTH - 1, SSD1306_HEIGHT - 1, White);
    ssd1306_line(SSD1306_WIDTH - 1, 1, 1, SSD1306_HEIGHT - 1, White);
    ssd1306_update_screen();
}

void ssd1306_TestRectangle()
{
    uint32_t delta;

    for (delta = 0; delta < 5; delta++)
    {
        ssd1306_draw_rectangle(1 + (5 * delta), 1 + (5 * delta), SSD1306_WIDTH - 1 - (5 * delta), SSD1306_HEIGHT - 1 - (5 * delta), White);
    }
    ssd1306_update_screen();
    return;
}

void ssd1306_TestFonts()
{
    ssd1306_fill(Black);
    ssd1306_set_cursor(2, 0);
    ssd1306_write_string("Font 16x26", Font_16x26, White);
    ssd1306_set_cursor(2, 26);
    ssd1306_write_string("Font 11x18", Font_11x18, White);
    ssd1306_set_cursor(2, 26 + 18);
    ssd1306_write_string("Font 7x10", Font_7x10, White);
    ssd1306_set_cursor(2, 26 + 18 + 10);
    ssd1306_write_string("Font 6x8", Font_6x8, White);
    ssd1306_update_screen();
}

void ssd1306_TestAll()
{
    ssd1306_init();
    ssd1306_TestFonts();
    // ssd1306_TestLine();
    // ssd1306_TestRectangle();
    // ssd1306_TestCircle();
}