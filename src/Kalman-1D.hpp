#include <stdint.h>

#ifndef __KALMAN_1D__
#define __KALMAN_1D__

class Kalman1d {
    public:
        Kalman1d(float processCov, float measCov, float estCov);
        ~Kalman1d();
        // 
        // Error function.
        float update(float meas);

    private:
        float q_; //process noise covariance
        float r_; //measurement noise covariance
        float p_; //estimation error covariance
        float x_; //value
        float k_; //kalman gain

};

#endif
