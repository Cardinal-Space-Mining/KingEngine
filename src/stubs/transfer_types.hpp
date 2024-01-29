#pragma once
#include <cstdint>

//Location of robot. Units are in M
typedef struct locationF_t{
    float x_m;
    float y_m;
}locationF_t;

namespace KingEngine{
    //Location of robot. Units are in mm. For internal use only
    typedef struct locationI_t{
        int32_t x_mm;
        int32_t y_mm;
    }locationI_t;

    inline locationF_t from_integer_location(locationI_t local){
        float x_mm_f = local.x_mm;
        float y_mm_f = local.y_mm;
        return {x_mm_f / 1000.0f, y_mm_f / 1000.0f};
    }

    inline locationI_t from_float_location(locationF_t local){
        int32_t x_m_i = local.x_m * 1000;
        int32_t y_m_i = local.y_m * 1000;
        return {x_m_i, y_m_i};
    }
}