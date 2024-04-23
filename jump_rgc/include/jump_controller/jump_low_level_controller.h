#ifndef JUMP_LOW_LEVEL_CONTROLLER_H
#define JUMP_LOW_LEVEL_CONTROLLER_H

#include <memory>
#include <set>
#include <string>

class LowLevelController
{
public:
    /// Constructor
    LowLevelController();

    /// Destructor
    ~LowLevelController();

    void Configure(double _Kp, double _Kd);

    double ComputeControl(double _q, double _qr, double _dq, double _dqr);

    double ComputeCompG();

private:
    int Kp = 300;
    int Kd = 20;
};
#endif