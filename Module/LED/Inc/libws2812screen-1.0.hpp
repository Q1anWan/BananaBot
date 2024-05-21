#pragma once
#ifndef LIB_WS2812SCREEN_HPP
#define LIB_WS2812SCREEN_HPP

#include "cstdint"
#include "libspi-a-1.0.h"

/*----
 *                  High
 *                    y
 *                    ^
 *                    |
 *                    |
 * Width x <----------o
 * */
namespace Screen {

    static uint8_t RED[3] = {255, 0, 0};
    static uint8_t GREEN[3] = {0, 255, 0};
    static uint8_t YELLOW[3] = {255, 255, 0};
    static uint8_t BLUE[3] = {0, 0, 255};
    static uint8_t BLACK[3] = {0, 0, 0};
    static uint8_t WHITE[3] = {255, 255, 255};

    class Display {
    protected:
        uint8_t *_frame_buff = nullptr;
        uint32_t _frame_buff_len;
        uint32_t _pixel_high;
        uint32_t _pixel_width;
        SPIA::cSPIA *_spi = nullptr;
    public:
        /*
         * @param pspi: SPI handle
         * @param frame_buff: Frame buffer = 9*PixelWidth*PixelHigh+2
         * @param pixel_width: Pixel width
         * */
        Display(SPIA::cSPIA *pspi, uint8_t *frame_buff, uint32_t pixel_width,
                uint32_t pixel_high);

        Display() = default;

        uint8_t ResetScreen(SPIA::cSPIA *pspi, uint8_t *frame_buff, uint32_t pixel_width,
                            uint32_t pixel_high);

        uint8_t SetPixel(uint32_t x, uint32_t y, uint8_t *rgb);

        void SetAllPixel(uint8_t *rgb);

        void SetAllPixel(uint8_t *rgb, float brightness);

        void Refresh();

        uint32_t GetWidth() { return _pixel_width; }

        uint32_t GetHigh() { return _pixel_high; }

        virtual void AsynchBuff();
    };
}
#endif