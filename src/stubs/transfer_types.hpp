#pragma once
#include <cstdint>

//Location of robot. Units are in M
typedef struct locationF_t{
    float x_m;
    float y_m;
}locationF_t;

namespace KingEngine{


    constexpr int32_t SCALE_FACTOR = 100000;
    //Location of robot. Units are in mm. For internal use only
    typedef struct locationI_t{
        int32_t x_mm;
        int32_t y_mm;
    }locationI_t;

    inline locationF_t from_integer_location(locationI_t local){
        float x_mm_f = local.x_mm;
        float y_mm_f = local.y_mm;
        return {x_mm_f / SCALE_FACTOR, y_mm_f / SCALE_FACTOR};
    }

    inline locationI_t from_float_location(locationF_t local){
        int32_t x_m_i = local.x_m * SCALE_FACTOR;
        int32_t y_m_i = local.y_m * SCALE_FACTOR;
        return {x_m_i, y_m_i};
    }
}
