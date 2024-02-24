#include <vector>
#include <numeric>
#include "king_engine/subsystems/Collection.h"
#include "king_engine/subsystems/subsystem_util.h"
#include "king_engine/subsystems/Hopper.h"
// This class is for the mining segment
// Sets a speed
// Starts based on Hopper position
// Algorithm to keep track of resistance in belt


void Collection::periodic() {
    
}

bool Collection::getTrencherJammed() {
    return trencherIsJammed;
}

void Collection::setTrencherJammed(bool status) {
    trencherIsJammed = status;
}

double Collection::getTrencherEncoder() {
    return trencherEncoder;
}

double Collection::getTrencherVelocity() {
    return trencherSpeed;
}

double Collection::getTrencherCurrent() {
    return trencherCurrent;
}

double Collection::getTrencherAvgCurrent() {
    return trench_avg_current;
}

