#ifndef COMPONENTS_SSD1306_SSD1306_FONTS
#define COMPONENTS_SSD1306_SSD1306_FONTS

#include <stdint.h>
#include "ssd1306_conf.h"

typedef struct 
{
    const uint8_t FontWidth;
    uint8_t FontHeight;
    const uint16_t *data;
} FontDef;

#ifdef SSD1306_INCLUDE_FONT_6x8
extern FontDef Font_6x8;
#endif
#ifdef SSD1306_INCLUDE_FONT_7x10
extern FontDef Font_7x10;
#endif
#ifdef SSD1306_INCLUDE_FONT_11x18
extern FontDef Font_11x18;
#endif
#ifdef SSD1306_INCLUDE_FONT_16x26
extern FontDef Font_16x26;
#endif

#endif /* COMPONENTS_SSD1306_SSD1306_FONTS */
