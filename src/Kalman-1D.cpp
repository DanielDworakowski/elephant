#include "Kalman-1D.hpp"

// 
// Apparently, if you delete these comments its stops compiling...........
// {
// }

Kalman1d::Kalman1d(float processCov, float measCov, float estCov)
    : q_(processCov)
    , r_(measCov)
    , p_(estCov)
    , x_(0.0)
{
}

Kalman1d::~Kalman1d()
{
}

float Kalman1d::update(float meas)
{
    //prediction update
    //omit x = x
    p_ = p_ + q_;

    //measurement update
    k_ = p_ / (p_ + r_);
    x_ = x_ + k_ * (meas - x_);
    p_ = (1 - k_) * p_;
    return x_;
}