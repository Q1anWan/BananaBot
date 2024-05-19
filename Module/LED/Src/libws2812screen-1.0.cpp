#include "libws2812screen-1.0.hpp"
#include <cstring>
using namespace Screen;

Display::Display(SPIA::cSPIA *pspi, uint8_t *frame_buff, uint32_t pixel_width,
                 uint32_t pixel_high) : _pixel_high(pixel_high), _pixel_width(pixel_width), _frame_buff(frame_buff),
                                        _spi(pspi), _frame_buff_len(2 + 9 * pixel_width * pixel_high) {
    if (frame_buff != nullptr) {
        _frame_buff = frame_buff;
    }
    if (pspi != nullptr) {
        _spi = pspi;
    }
    memset(_frame_buff, 0, _frame_buff_len);
}
uint8_t Display::ResetScreen(SPIA::cSPIA *pspi, uint8_t *frame_buff, uint32_t pixel_width,
                    uint32_t pixel_high){
    if ((pspi != nullptr)&&(frame_buff != nullptr)){
        _frame_buff = frame_buff;
        _spi = pspi;
    }
    else{
        return 1;
    }
    _pixel_high = pixel_high;
    _pixel_width = pixel_width;
    _frame_buff_len = 2 + 9 * pixel_width * pixel_high;
    memset(_frame_buff, 0, _frame_buff_len);
    return 0;
}

void Display::Refresh() {
    AsynchBuff();
    _spi->TransmitDMA(_frame_buff, _frame_buff_len);
}

uint8_t Display::SetPixel(uint32_t x, uint32_t y, uint8_t *rgb) {
    if ((x >= _pixel_width) || (y >= _pixel_high) || (rgb == nullptr)) {
        return 1;
    }
    uint8_t rgb_t[3] = {rgb[1], rgb[0], rgb[2]};
    uint32_t index = 1 + (x + y * _pixel_width) * 9;

    for (uint8_t i = 0; i < 3; i++) {
        uint32_t tmp = 0;
        uint8_t cmp = 0x80;
        for (uint8_t j = 0; j < 8; j++) {
            tmp |= ((rgb_t[i] & cmp) ? 0b110 : 0b100);
            cmp >>= 1;
            if (j < 7) { tmp <<= 3; }
        }
        _frame_buff[index + 3 * i] = (uint8_t) ((tmp >> 16) & 0xFF);
        _frame_buff[index + 3 * i + 1] = (uint8_t) ((tmp >> 8) & 0xFF);
        _frame_buff[index + 3 * i + 2] = (uint8_t) ((tmp) & 0xFF);
    }
    return 0;
}

void Display::SetAllPixel(uint8_t *rgb) {
    for (uint32_t i = 0; i < _pixel_width; ++i) {
        for (uint32_t j = 0; j < _pixel_high; ++j) {
            SetPixel(i, j, rgb);
        }
    }
}

void Display::SetAllPixel(uint8_t *rgb, float brightness) {
    if(rgb == nullptr){
        return;
    }
    uint8_t rgb_t[3];
    for (uint8_t i = 0; i < 3; i++) {
        rgb_t[i] = (uint8_t) (rgb[i] * brightness);
    }
    for (uint32_t i = 0; i < _pixel_width; ++i) {
        for (uint32_t j = 0; j < _pixel_high; ++j) {
            SetPixel(i, j, rgb_t);
        }
    }
}

void Display::AsynchBuff(){
    //Do sth
}