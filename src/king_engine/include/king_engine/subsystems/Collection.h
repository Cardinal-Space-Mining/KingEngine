#pragma once

#include <vector>


class Collection {
    private:
        double trencherEncoder = 0;
        double trencherSpeed = 0;
        double trencherCurrent = 0;
        double trencherVoltage = 0;
        double busVoltage = 0;
        bool hasZeroed = false;
        bool trencherIsJammed = false;
        std::vector<double> trencherCurrentList;
        int movingAvgRange = 10;
        double trench_avg_current = 0.0;
        int miningStartPosition;
        int sortieCount = 0;
        int trench;
    public:
        void periodic();
        bool getTrencherJammed();
        void setTrencherJammed(bool status);
        double getTrencherEncoder();
        double getTrencherVelocity();
        double getTrencherCurrent();
        double getTrencherAvgCurrent();
};
