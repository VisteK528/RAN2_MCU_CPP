#ifndef STM32_TFT_LCD_H
#define STM32_TFT_LCD_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define LCD_WIDTH	160
#define LCD_HEIGHT	128

#define BLACK     0x0000
#define RED       0xf800
#define GREEN     0x07e0
#define BLUE      0x001f
#define YELLOW    0xffe0
#define MAGENTA   0xf81f
#define CYAN      0x07ff
#define WHITE     0xffff

void lcd_init(void);
void lcd_transfer_done(void);
bool lcd_is_busy(void);
void lcd_fill_box(int x, int y, int width, int height, uint16_t color);
void lcd_put_pixel(int x, int y, uint16_t color);
uint16_t lcd_get_pixel(int x, int y);
void lcd_draw_image(int x, int y, int width, int height, const uint8_t* data);
size_t lcd_flush(void);

#ifdef __cplusplus
}
#endif

#endif //STM32_TFT_LCD_H
