#ifndef RGC1_H
#define RGC1_H

#include "jump_controller/rgc.h"
#include "jump_controller/jump_robot_model.h"

class RGC1 : public RGC
{
public:
    RGC1(JumpRobot *Robot);

    ~RGC1();

    void UpdateDynamicModel() override;

    void UpdateModelConstants() override;

    JumpRobot *RobotKin;

    Eigen::MatrixXd C_cons_aux, GRF_mtx;

    Eigen::Matrix<double, 2, 1> n1;
    Eigen::Matrix<double, 2, 1> t1;
};

#endif