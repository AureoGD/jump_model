#include "jump_controller/jump_interface.h"

bool JumpInterface::srvEcho(const jump::msgs::ResquestAction &_req,
                            gz::msgs::Int32 &_rep)
{
  auto response = _req.type();

  if (response == 1)
  {
    hal::ReadMatrix(_req.mtx(), &StatesMatrix);
    this->OnEvent();
    _rep.set_data(this->action);
  }
  else
  {

    hal::ReadVector(_req.vec(), &StatesVector);
    this->OnEvent();
    _rep.set_data(this->action);
  }

  return true;
}

int JumpInterface::OnEvent()
{
  // PyGILState_STATE state = PyGILState_Ensure();
  py::object result = _cbEvent.call();
  return 1;
  // PyGILState_Release(state);
}

JumpInterface::JumpInterface(const std::string &service_name, py::object cb_func)
{
  this->service = service_name;

  std::cout << "List of network interfaces in this machine:" << std::endl;
  for (const auto &netIface : gz::transport::determineInterfaces())
    std::cout << "\t" << netIface << std::endl;

  if (this->_Node.Advertise<JumpInterface, jump::msgs::ResquestAction, gz::msgs::Int32>(this->service, &JumpInterface::srvEcho, this))
  {
    std::cout << "The service [" << this->service << "] was created" << std::endl;
  }
  else
  {
    std::cout << "Error advertising service [" << this->service << "]" << std::endl;
  }

  this->_cbEvent = cb_func;
}

JumpInterface::~JumpInterface()
{
}