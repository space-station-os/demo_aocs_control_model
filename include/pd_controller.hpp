#ifndef _PD_CONTROLLER_H
#define _PD_CONTROLLER_H

#include <iostream>
#include <cmath>

class pd_controller {
    public:
        pd_controller();
    
    private:
        float outDelta[4];
        float setDelta[4];
        float kp;
        float kd;

    
};

#endif //_PD_CONTROLLER_H