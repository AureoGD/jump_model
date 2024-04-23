#ifndef JUMP_INTERFACE_H
#define JUMP_INTERFACE_H

#include <iostream>
#include <string>

// #include <osqp.h>
#include <OsqpEigen/OsqpEigen.h>
#include <gz/sim/System.hh>
#include <gz/transport.hh>
#include <gz/msgs.hh>
#include <pybind11/pybind11.h>

#include "jump/msgs/matrix.pb.h"
#include "jump/msgs/ResquestAction.pb.h"
#include "jump_controller/matrix.h"

namespace hal = hal;
namespace py = pybind11;

class JumpInterface
{
public:
  /// Constructor
  JumpInterface(const std::string &service_name, py::object cb_func);

  /// Destructor
  virtual ~JumpInterface();

  // call back
  // bool srvEcho(const gz::msgs::Int32 &_req, gz::msgs::Int32 &_rep);
  bool srvEcho(const jump::msgs::ResquestAction &_req, gz::msgs::Int32 &_rep);

  int OnEvent();

  int action = -1;

  bool request = false;

  py::object _cbEvent;

  gz::transport::Node _Node;

  std::string service;

  Eigen::MatrixXd StatesMatrix;
  Eigen::VectorXd StatesVector;
};
#endif