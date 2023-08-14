#include "../include/hagl_hal.h"

static void put_pixel(void* self, int16_t x, int16_t y, hagl_color_t color){
    lcd_put_pixel(x, y, color);
}

static void hline(void *self, int16_t x0, int16_t y0, uint16_t width, hagl_color_t color)
{
    for(int16_t i = 0; i < width; i++){
        lcd_put_pixel(x0+i, y0, color);
    }
}

static void vline(void *self, int16_t x0, int16_t y0, uint16_t height, hagl_color_t color)
{
    for(int16_t i = 0; i < height; i++){
        lcd_put_pixel(x0, y0+i, color);
    }
}

static hagl_color_t get_pixel(void* self, int16_t x0, int16_t y0){
    return lcd_get_pixel(x0, y0);
}

static size_t flush(void* self){
    return lcd_flush();
}

void hagl_hal_init(hagl_backend_t* backend){
    lcd_init();

    backend->height = LCD_HEIGHT;
    backend->width = LCD_WIDTH;
    backend->depth = 16;

    backend->put_pixel = put_pixel;
    backend->hline = hline;
    backend->vline = vline;
    backend->get_pixel = get_pixel;
    backend->flush = flush;
}