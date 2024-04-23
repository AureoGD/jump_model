
set(BINDINGS_MODULE_NAME "jump_interface_py")

pybind11_add_module(${BINDINGS_MODULE_NAME} MODULE
    "../include/jump_controller/jump_interface.h"    
    "../src/jump_interface.cpp"
    "./src/jump_interface_pybind11.cpp"
)

target_link_libraries(${BINDINGS_MODULE_NAME}
	PUBLIC
		${GZ-SIM_LIBRARIES}
		${GZ-MATH_LIBRARIES}
		${GZ-COMMON_LIBRARIES}
		${GZ-SENSORS_LIBRARIES}
		osqp::osqp
		OsqpEigen::OsqpEigen
		jump_msgs-msgs
)
