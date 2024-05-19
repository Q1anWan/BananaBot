#pragma once
#ifndef TASKWHEEL_H
#define TASKWHEEL_H

#include "cstdint"

namespace Wheel {
// We use Right-Forward.
// Right-Forward is one.
// Anti-Clockwise is positive.
// Forward-Left|Right-Forward
//        0 -- 1 +-
//        1 ++ 0 -+
//        2 -+ 3 ++
//        3 +- 2 --
//
//                Y
//      1---a---0 |
//      |       | |
//      b       b  -----X
//      |       |
//      2---a---3
//

#define CHS_A_PLUS_B 0.63f
#define RPM_CONST 9167.3247220931713402877047702568f            // m/s -> rpm
#define RPM_CONST_REV 0.00010908307824964559855773067303054f    // rpm -> m/s

    struct __attribute__((packed, aligned(1))) M2006_t {
        uint16_t mechanical_degree;
        int16_t rpm;
        int16_t torque;
    };

}

#endif