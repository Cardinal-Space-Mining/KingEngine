#include <vector>
double getMovingAverage(std::vector<double>& motorDataList, int movingAvgRange) {
    double currentAverage = 0.0;
    if (motorDataList.size() > movingAvgRange) {
        motorDataList.erase(motorDataList.begin());
    }
    currentAverage = std::accumulate(motorDataList.begin(), motorDataList.end(), currentAverage);
    currentAverage /= motorDataList.size();
    return currentAverage;
}