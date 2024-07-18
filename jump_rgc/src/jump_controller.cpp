#include "jump_controller/jump_controller.h"
#include "jump_controller/jump_low_level_controller.h"
#include "jump_controller/jump_rgc.h"
#include "jump_controller/matrix.h"
#include "jump/msgs/matrix.pb.h"

#include <gz/msgs/model.pb.h>

#include <string>
#include <vector>

#include <gz/plugin/Register.hh>

#include "gz/sim/components/ChildLinkName.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/JointAxis.hh"
#include "gz/sim/components/JointForce.hh"
#include "gz/sim/components/JointPosition.hh"
#include "gz/sim/components/JointVelocity.hh"
#include "gz/sim/components/JointForceCmd.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/ParentLinkName.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/JointType.hh"
#include "gz/sim/Util.hh"
#include "gz/sim/Model.hh"
#include <gz/math/PID.hh>

#include <gz/msgs.hh>
#include <gz/transport.hh>

namespace hal = hal;

JumpController::JumpController()
    : jointPos(Eigen::Matrix<double, 2, 1>::Zero()),
      jointVel(Eigen::Matrix<double, 2, 1>::Zero()),
      jointPRef(Eigen::Matrix<double, 2, 1>::Zero()),
      basePos(Eigen::Matrix<double, 2, 1>::Zero()),
      baseVel(Eigen::Matrix<double, 2, 1>::Zero()),
      _simpleRGC(&_JumpRobot, &jointPos, &jointVel, &jointPRef),
      _JumpRobot(&jointPos, &jointVel, &basePos, &baseVel),
      _JumpRGC(&_JumpRobot, &jointPos, &jointVel, &jointPRef)
{
  gzmsg << "#### LOADING JUMP CONTROLLER #####" << std::endl;
  this->jointPRef = {-0.75, 0.75};
  this->jointVRef = {0.0, 0.0};
  gzmsg << "#### LOADED JUMP CONTROLLER PLUGIN #####" << std::endl;
}

JumpController::~JumpController()
{
}

void JumpController::Configure(const gz::sim::Entity &_entity,
                               const std::shared_ptr<const sdf::Element> &_sdf,
                               gz::sim::EntityComponentManager &_ecm,
                               gz::sim::EventManager &_eventMgr)
{
  gzmsg << "#### CONFIGURATION OF JUMP-RGC PLUGIN #####" << std::endl;

  this->model = gz::sim::v8::Model(_entity);

  if (!this->model.Valid(_ecm))
  {
    gzerr << "RGC plugin should be attached to a model "
          << "entity. Failed to initialize." << std::endl;
    return;
  }

  if (_sdf->HasElement("joint_name"))
  {
    auto elem = _sdf->FindElement("joint_name");
    while (elem)
    {
      std::string jointName = elem->Get<std::string>();
      this->jointNames.push_back(jointName); // not used
      gz::sim::v8::Entity jointEntity = this->model.JointByName(_ecm, jointName);

      if (jointEntity != gz::sim::v8::kNullEntity)
      {
        this->CreateComponents(_ecm, jointEntity, jointName);
      }
      else
      {
        gzerr << "Joint with name[" << jointName << "] not found. "
              << "The JointStatePublisher will not publish this joint.\n";
      }

      elem = elem->GetNextElement("joint_name");
    }
  }
  else
  {
    gzerr << "Atribute 'joint_name' was not found in the SDF file .\n";
  }

  // TODO - create a function to positioning the model at a inicial position

  // Configure Low Level Controller - llc

  double rate_lc = _sdf->Get<double>("lc_update_rate", 10).first;

  std::chrono::duration<double> period_lc{rate_lc > 0 ? 1 / rate_lc : 0};
  this->lc_updatePeriod =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(period_lc);

  gzmsg << "Low Level Controller update rate: " << rate_lc << " Hz" << std::endl;

  this->llc.Configure(this->Kp, this->Kd);
  this->_JumpRGC.RGCConfig(1.0 / rate_lc, this->Kp, this->Kd);
  this->_simpleRGC.RGCConfig(1.0 / rate_lc, this->Kp, this->Kd);

  // Configure MPC controller

  double rate = _sdf->Get<double>("rgc_update_rate", 10).first;

  std::chrono::duration<double> period{rate > 0 ? 1 / rate : 0};
  this->rgc_updatePeriod =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);

  gzmsg << "Reference Governor Control update rate: " << rate << " Hz" << std::endl;

  // create the "RobotInfos" service
  if (this->_Node.Advertise<JumpController, gz::msgs::Boolean, jump::msgs::VectorMsg>(this->service, &JumpController::srvEcho, this))
  {
    gzmsg << "The service [" << '/' << this->service << "] was created" << std::endl;
  }
  else
  {
    gzmsg << "Error advertising service [" << '/' << this->service << "]" << std::endl;
  }

  // create "robot state publisher"
  this->modelSTPub = std::make_unique<gz::transport::Node::Publisher>(this->_Node.Advertise<jump::msgs::VectorMsg>(this->mp_topic));
  if (!this->modelSTPub)
    gzmsg << "Error to create the topic [" << this->mp_topic << "]" << std::endl;
  else
    gzmsg << "The topic [" << this->mp_topic << "] was created" << std::endl;

  // Subscribe to the touch plugin topic
  if (this->_Node.Subscribe<JumpController, gz::msgs::Wrench>(this->ts_topic, &JumpController::TouchCB, this))
  {
    gzmsg << "Subscribed to topic [" << this->ts_topic << "]" << std::endl;
  }

  gzmsg << "#### RGC-PLUGIN CONFIGURATION IS DONE #####" << std::endl;

  StatesList[0] = &basePos;
  StatesList[1] = &baseVel;
  StatesList[2] = &this->_JumpRobot.com_pos;
  StatesList[3] = &this->_JumpRobot.com_vel;
  StatesList[4] = &this->_JumpRobot.foot_pos;
  StatesList[5] = &this->_JumpRobot.foot_vel;
  StatesList[6] = &jointPos;
  StatesList[7] = &jointVel;
  StatesList[8] = &Effort_cmd;
  StatesList[9] = &(this->_JumpRGC.refHL);
  StatesList[10] = &jointPRef;

  NNStatesList[0] = &this->_JumpRobot.com_pos;
  NNStatesList[1] = &this->_JumpRobot.com_vel;
  NNStatesList[2] = &this->_JumpRobot.foot_pos;
  NNStatesList[3] = &this->_JumpRobot.foot_vel;
  NNStatesList[4] = &jointPos;
  NNStatesList[5] = &jointPRef;

  if (this->states_type)
    this->req.set_type(0);
  else
    this->req.set_type(1);
}

void JumpController::CreateComponents(gz::sim::v8::EntityComponentManager &_ecm,
                                      gz::sim::v8::Entity _joint,
                                      std::string _joint_name)
{

  if (!_ecm.EntityHasComponentType(_joint, gz::sim::v8::components::Joint::typeId))
  {
    gzerr << "Entity with name[" << _joint_name
          << "] is not a joint\n";
  }
  else
  {
    gzmsg << "Identified joint [" << _joint_name
          << "] as Entity [" << _joint << "]\n";
  }

  this->joints.push_back(_joint);
  // Create joint position component if one doesn't exist
  if (!_ecm.EntityHasComponentType(_joint,
                                   gz::sim::v8::components::JointPosition().TypeId()))
  {
    _ecm.CreateComponent(_joint, gz::sim::v8::components::JointPosition());
    gzmsg << "Create a 'Position Component' for the joint "
          << _joint_name << ", component: "
          << _joint << std::endl;
  }

  // Create joint velocity component if one doesn't exist
  if (!_ecm.EntityHasComponentType(_joint,
                                   gz::sim::v8::components::JointVelocity().TypeId()))
  {
    _ecm.CreateComponent(_joint, gz::sim::v8::components::JointVelocity());
    gzmsg << "Create a 'Velocity Component' for the joint "
          << _joint_name << ", component: "
          << _joint << std::endl;
  }

  if (!_ecm.EntityHasComponentType(_joint,
                                   gz::sim::v8::components::JointForceCmd().TypeId()))
  {
    _ecm.CreateComponent(_joint, gz::sim::v8::components::JointForceCmd({0}));
    gzmsg << "Create a 'Force Component' for the joint "
          << _joint_name << ", component: "
          << _joint << std::endl;
  }
}

void JumpController::PreUpdate(const gz::sim::UpdateInfo &_info,
                               gz::sim::EntityComponentManager &_ecm)
{
  if (this->joints.empty())
    return;

  // Nothing left to do if paused.
  if (_info.paused)
    return;

  // Read joints position and velocity
  this->ReadSensors(_ecm);

  // Verify if is time to update the reference
  auto elapsed_rgc = _info.simTime - this->lastUpdateTime_rgc;

  if (elapsed_rgc > std::chrono::steady_clock::duration::zero() && elapsed_rgc >= this->rgc_updatePeriod)
  {
    // update model matrices
    this->_JumpRobot.UpdateSysMatrices();

    // requestes the ChooseAction service
    // first update the NN input data
    if (this->states_type)
    {
      this->req.mutable_vec()->Clear();
      // for (int i = 0; i < 6; i++)
      //   hal::WriteVector(*this->NNStatesList[i], this->req.mutable_vec());

      hal::WriteVector(this->_JumpRobot.com_pos, this->req.mutable_vec());
      hal::WriteVector(this->_JumpRobot.com_vel, this->req.mutable_vec());
      hal::WriteVector(this->_JumpRobot.foot_pos, this->req.mutable_vec());
      hal::WriteVector(this->_JumpRobot.foot_vel, this->req.mutable_vec());
      hal::WriteVector(jointPos, this->req.mutable_vec());
      hal::WriteVector(jointPRef, this->req.mutable_vec());
    }
    else
    {
      this->req.set_type(1);
      this->req.clear_mtx();
    }

    // this->_simpleRGC.ChooseRGCPO(1);

    // call the service
    // bool executed = _Node.Request("/ChoseAction", this->req, 5, this->res, result);
    // // call the PO using the response of the service
    // this->const_retun = this->_JumpRGC.ChooseRGCPO(this->res.data());
    // std::cout << this->res.data() << std::endl;
    // this->const_retun = this->_simpleRGC.ChooseRGCPO(this->res.data());

    if (teste < 1000)
    {
      // this->jointPRef << -3.14 * 60 / 180, 3.14 * 120 / 180;
      this->_simpleRGC.ChooseRGCPO(0);
      // std::cout << "mode 1" << std::endl;
    }
    else if (teste >= 1000 && teste < 1250)
    {
      // this->jointPRef << -3.14 * 30 / 180, 3.14 * 60 / 180;
      this->_simpleRGC.ChooseRGCPO(1);
      // std::cout << "mode 0 " << std::endl;
    }
    else if (teste >= 1250 && teste < 1500)
    {
      // this->jointPRef << -3.14 * 30 / 180, 3.14 * 60 / 180;
      this->_simpleRGC.ChooseRGCPO(5);
      // std::cout << "mode 4 " << std::endl;
    }
    else
      teste = 1000;
    std::cout << teste << std::endl;
    teste++;
    // std::cout << valor << std::endl;

    // std::cout << *this->StatesList[9] << std::endl;
    // std::cout << this->_JumpRGC.refHL << std::endl;
    //  std::cout<<this->jointVel<<std::endl;
    //  save the current time for the next interation
    this->lastUpdateTime_rgc = _info.simTime;
  }

  auto elapsed = _info.simTime - this->lastUpdateTime_lc;

  if (elapsed > std::chrono::steady_clock::duration::zero() && elapsed < this->lc_updatePeriod)
    return;

  for (int index = 0; index < 2; index++)
  {
    this->Effort_cmd(index) = this->llc.ComputeControl(this->jointPos(index),
                                                       this->jointPRef(index),
                                                       this->jointVel(index),
                                                       this->jointVRef(index));

    auto forceComp = _ecm.Component<gz::sim::v8::components::JointForceCmd>(this->joints[index + 1]);

    *forceComp = gz::sim::v8::components::JointForceCmd({this->Effort_cmd(index)});
    // gzmsg << "joint effort: " << this->Effort_cmd(index) << std::endl;
  }
  this->lastUpdateTime_lc = _info.simTime;
}

void JumpController::ReadSensors(gz::sim::EntityComponentManager &_ecm)
{
  std::lock_guard<std::mutex> lock(this->JumpControllerMutex);
  for (int index = 0; index < 3; index++)
  {
    const auto *jointPositions = _ecm.Component<gz::sim::v8::components::JointPosition>(this->joints[index]);
    const auto *jointVelocity = _ecm.Component<gz::sim::v8::components::JointVelocity>(this->joints[index]);

    if (jointPositions == nullptr || jointPositions->Data().empty() || jointVelocity == nullptr || jointVelocity->Data().empty())
      return;

    if (index == 0)
    {
      // TODO - edit the SDF file
      this->basePos[0] = 0;
      this->basePos[1] = 0.75 + jointPositions->Data()[0];

      this->baseVel[0] = 0;
      this->baseVel[1] = jointVelocity->Data()[0];
    }
    else
    {
      this->jointPos(index - 1) = jointPositions->Data()[0];
      this->jointVel(index - 1) = jointVelocity->Data()[0];
    }
  }
}

void JumpController::PostUpdate(const gz::sim::UpdateInfo &_info,
                                const gz::sim::EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  if (!this->modelSTPub)
    gzmsg << "ERROR" << std::endl;

  // Prepare the ModelStatesMsg with the new info
  this->ModelStatesMsg.Clear();

  // for (int i = 0; i <= 10; i++)
  // {
  //  hal::WriteVector(*this->StatesList[i], &this->ModelStatesMsg);
  //  std::cout << *this->StatesList[i] << std::endl;
  // }

  // brute force
  hal::WriteVector(basePos, &this->ModelStatesMsg);
  hal::WriteVector(baseVel, &this->ModelStatesMsg);
  hal::WriteVector(this->_JumpRobot.com_pos, &this->ModelStatesMsg);
  hal::WriteVector(this->_JumpRobot.com_vel, &this->ModelStatesMsg);
  hal::WriteVector(this->_JumpRobot.foot_pos, &this->ModelStatesMsg);
  hal::WriteVector(this->_JumpRobot.foot_vel, &this->ModelStatesMsg);
  hal::WriteVector(jointPos, &this->ModelStatesMsg);
  hal::WriteVector(jointVel, &this->ModelStatesMsg);
  hal::WriteVector(Effort_cmd, &this->ModelStatesMsg);
  hal::WriteVector(this->_simpleRGC.qhl, &this->ModelStatesMsg);
  hal::WriteVector(jointPRef, &this->ModelStatesMsg);
  this->ModelStatesMsg.add_data(static_cast<double>(this->const_retun));
  this->ModelStatesMsg.add_data(static_cast<double>(this->touch_st));

  // Publish the roobot state msg
  this->modelSTPub->Publish(this->ModelStatesMsg);
}

bool JumpController::srvEcho(const gz::msgs::Boolean &_req,
                             jump::msgs::VectorMsg &_rep)
{
  _rep.Clear();

  // for (int i = 0; i <= 10; i++)
  //   hal::WriteVector(*this->StatesList[i], &_rep);
  // _rep.add_data(static_cast<double>(this->const_retun));
  // _rep.add_data(static_cast<double>(this->touch_st));

  // brute force
  hal::WriteVector(basePos, &_rep);
  hal::WriteVector(baseVel, &_rep);
  hal::WriteVector(this->_JumpRobot.com_pos, &_rep);
  hal::WriteVector(this->_JumpRobot.com_vel, &_rep);
  hal::WriteVector(this->_JumpRobot.foot_pos, &_rep);
  hal::WriteVector(this->_JumpRobot.foot_vel, &_rep);
  hal::WriteVector(jointPos, &_rep);
  hal::WriteVector(jointVel, &_rep);
  hal::WriteVector(Effort_cmd, &_rep);
  hal::WriteVector(this->_JumpRGC.refHL, &_rep);
  hal::WriteVector(jointPRef, &_rep);
  _rep.add_data(static_cast<double>(this->const_retun));
  _rep.add_data(static_cast<double>(this->touch_st));
  return true;
}

void JumpController::TouchCB(const gz::msgs::Wrench &msg)
{
  if (abs(msg.force().z()) > 10)
    this->touch_st_cb = true;
  else
    this->touch_st_cb = false;
  this->touch_st = this->touch_st_cb * this->last_touch_state_cb;
  this->last_touch_state_cb = this->touch_st_cb;
}

GZ_ADD_PLUGIN(JumpController,
              gz::sim::System,
              JumpController::ISystemConfigure,
              JumpController::ISystemPreUpdate,
              JumpController::ISystemPostUpdate);

GZ_ADD_PLUGIN_ALIAS(JumpController, "JumpController")
