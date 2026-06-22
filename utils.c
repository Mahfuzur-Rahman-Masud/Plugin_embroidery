#include "grbl/grbl.h"
#include "grbl/system.h"
#include "grbl/gcode.h"


void get_raw_coordinates(float *mpos, float *wpos)
{
    // 1. Get raw Machine Position (MPos) directly from the global system state
    mpos[X_AXIS] = sys.position[X_AXIS];
    mpos[Y_AXIS] = sys.position[Y_AXIS];
    mpos[Z_AXIS] = sys.position[Z_AXIS];

// gc_get_offset

    // 2. Calculate Work Position (WPos)
    // sys.coord_offset holds the currently active coordinate system offset (G54-G59)
    // gc_state.g92_offset holds volatile G92 coordinate offsets if applied
    for (uint8_t idx = 0; idx < 3; idx++) {
        float total_offset = sys.coord_offset[idx];


        
        if (gc_state.g92_offset_applied) {
            total_offset += gc_state.g92_offset.xyz[idx];
        }
        
        wpos[idx] = sys.position[idx] - total_offset;
    }
}