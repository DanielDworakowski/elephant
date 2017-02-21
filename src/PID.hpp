#include <stdint.h>

#ifndef __PID__
#define __PID__

class PID {
    public:
        PID(float p, float i, float d, float max, float min);
        ~PID();
        // 
        // Error function.
        float getError(float ref, float meas);

    private:
        float p_, i_, d_;
        float lastErr_;
        uint32_t lastTime_;
        float integralErr_;
        float max_, min_;

};

#endif /* _PID_WRAPPER_H_ */
