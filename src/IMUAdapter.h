#pragma once

#include <imuFilter.h>

constexpr float GAIN = 0.5;

class IMUAdapter {
    private:
        imuFilter<&GAIN> fusion;

    public:
        float rot[3];
        IMUAdapter();
        void begin();
        void calibrateGyro();
        void calibrateAcc();
        void update();
        void getRPY(float rot[]);
        void printRotation();
};