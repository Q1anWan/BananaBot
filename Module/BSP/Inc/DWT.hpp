#pragma once
#ifndef DWT_HPP
#define DWT_HPP

#include <main.h>

class cDWT {
protected:
    volatile uint32_t _sys_clk;
    volatile uint32_t _start;
public:
    cDWT() {
        if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
            CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
            DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
        }
        _sys_clk = SystemCoreClock;
        _start = DWT->CYCCNT;
    }

    static inline uint32_t cnt() {
        return DWT->CYCCNT;
    }

    inline float dt_sec() {
        uint32_t tmp = DWT->CYCCNT - _start;
        _start = DWT->CYCCNT;
        return (float) tmp / (float) _sys_clk;
    }

    inline void update() {
        _start = DWT->CYCCNT;
    }
};

#endif

