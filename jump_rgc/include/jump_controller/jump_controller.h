#ifndef JUMP_CONTROLLER_H
#define JUMP_CONTROLLER_H

#include <memory>
#include <set>
#include <string>

#include <gz/sim/Model.hh>
#include <gz/transport/Node.hh>
#include <gz/sim/System.hh>
#include <osqp.h>
#include <OsqpEigen/OsqpEigen.h>
#include <gz/math/PID.hh>

#include <gz/msgs.hh>
#include <gz/transport.hh>

#include "jump_controller/jump_low_level_controller.h"
#include "jump_controller/jump_robot_model.h"
#include "jump_controller/jump_rgc.h"
#include "jump_controller/jump_interface.h"
#include "jump_controller/matrix.h"

#include "jump_controller/simple_rgc.h"

#include "jump/msgs/matrix.pb.h"
#include "jump/msgs/ResquestAction.pb.h"

namespace hal = hal;

class JumpController : public gz::sim::System,
                       public gz::sim::ISystemConfigure,
                       public gz::sim::ISystemPreUpdate,
                       public gz::sim::ISystemPostUpdate
{
public:
    /// Constructor
    JumpController();

    /// Destructor
    ~JumpController();

    /// Implement Configure callback, provided by ISystemConfigure
    /// and called once at startup.
    void Configure(const gz::sim::Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   gz::sim::EntityComponentManager &_ecm,
                   gz::sim::EventManager &_eventMgr);

    /// Implement PreUpdate callback, provided by ISystemPreUpdate
    /// and called at every iteration, before physics is done
    void PreUpdate(const gz::sim::UpdateInfo &_info,
                   gz::sim::EntityComponentManager &_ecm);

    /// Implement PostUpdate callback, provided by ISystemPostUpdate
    /// and called at every iteration, after physics is done
    void PostUpdate(const gz::sim::UpdateInfo &_info,
                    const gz::sim::EntityComponentManager &_ecm);

    void CreateComponents(gz::sim::EntityComponentManager &_ecm,
                          gz::sim::v8::Entity _joint,
                          std::string _joint_name);

    void ReadSensors(gz::sim::EntityComponentManager &_ecm);

    void TouchCB(const gz::msgs::Wrench &msg);

    bool srvEcho(const gz::msgs::Boolean &_req, jump::msgs::VectorMsg &_rep);

private:
    LowLevelController llc;

public:
    JumpRobot _JumpRobot;

public:
    JumpRGC _JumpRGC;

public:
    SimpleRGC _simpleRGC;

private:
    std::mutex JumpControllerMutex;

public:
    std::vector<std::string> jointNames;

private:
    gz::sim::v8::Model model;

private:
    gz::transport::Node _Node;

private:
    jump::msgs::ResquestAction req;

private:
    gz::msgs::Int32 res;

    bool result;
    unsigned int timeout = 5000;

private:
    std::string service = "RobotLearningStates";

    // Joint Entity
public:
    std::vector<gz::sim::v8::Entity>
        joints;

public:
    std::chrono::steady_clock::duration rgc_updatePeriod{0};

public:
    std::chrono::steady_clock::duration lastUpdateTime_rgc{0};

public:
    std::chrono::steady_clock::duration lc_updatePeriod{0};

public:
    std::chrono::steady_clock::duration lastUpdateTime_lc{0};

    // double base_pos;
    // double base_vel;

    Eigen::Matrix<double, 2, 1> basePos;
    Eigen::Matrix<double, 2, 1> baseVel;

    Eigen::Matrix<double, 2, 1> jointPos;
    Eigen::Matrix<double, 2, 1> jointVel;

    Eigen::Matrix<double, 2, 1> jointPRef;
    Eigen::Matrix<double, 2, 1> jointVRef;

    Eigen::Matrix<double, 2, 1> Effort_cmd;

    int Kp = 300;
    int Kd = 20;

    bool states_type = true;

    bool const_retun = false;
    std::string ts_topic = "/JumpRobot/foot/force_torque";
    std::string ts_servece = "/JumpRobotTouchSensor/enable";
    gz::msgs::Boolean ts_req;
    int ts_timeout = 10;
    bool touch_st = false;
    bool touch_st_cb = false;
    bool last_touch_state_cb = false;

    Eigen::MatrixXd NNStates;

    int npo = -1;

private:
    std::unique_ptr<gz::transport::Node::Publisher> modelSTPub;
    std::string mp_topic = "/JumpRobot/ModelStates";
    jump::msgs::VectorMsg ModelStatesMsg;

    Eigen::Matrix<double, 2, 1> *StatesList[9];
    Eigen::Matrix<double, 2, 1> *NNStatesList[6];
    // std::vector<Eigen::Matrix<double, 2, 1>> *Stateslist[9];

    int teste = 0;
};
#endif