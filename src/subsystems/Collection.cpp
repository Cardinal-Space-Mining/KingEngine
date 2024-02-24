#include <vector>
#include <numeric>
#include "Collection.h"
#include "subsystem_util.h"
#include "Hopper.h"
// This class is for the mining segment
// Sets a speed
// Starts based on Hopper position
// Algorithm to keep track of resistance in belt


class Collection {
    Hopper hopper;
}

void periodic() {
    
}

bool getTrencherJammed() {
    return trencherIsJammed;
}

void setTrencherJammed(boolean status) {
    trencherIsJammed = status;
}

double getTrencherEncoder() {
    return trencherEncoder;
}

double getTrencherVelocity() {
    return trencherSpeed;
}

double getTrencherCurrent() {
    return trencherCurrent;
}

double getTrencherAvgCurrent() {
    return trench_avg_current;
}

