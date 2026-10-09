cmake_minimum_required(VERSION 3.11)
project(unitree_hg)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
execute_process(
	COMMAND "${Python3_EXECUTABLE}" -I -c "import numpy; print(numpy.get_include())"
	RESULT_VARIABLE NUMPY_QUERY_RESULT
	OUTPUT_VARIABLE DETECTED_NUMPY_INCLUDE_DIR
	OUTPUT_STRIP_TRAILING_WHITESPACE
)
if(NOT NUMPY_QUERY_RESULT EQUAL 0 OR NOT DETECTED_NUMPY_INCLUDE_DIR)
	message(FATAL_ERROR "Could not query NumPy include path from ${Python3_EXECUTABLE}")
endif()

# rosidl_generator_py still requests the legacy FindPython variables. Reuse the
# Python installation already resolved by FindPython3 (important in Conda).
set(Python_EXECUTABLE ${Python3_EXECUTABLE} CACHE FILEPATH "" FORCE)
set(Python_INCLUDE_DIRS ${Python3_INCLUDE_DIRS} CACHE STRING "" FORCE)
set(Python_LIBRARIES ${Python3_LIBRARIES} CACHE STRING "" FORCE)
set(Python_NumPy_INCLUDE_DIR ${DETECTED_NUMPY_INCLUDE_DIR} CACHE PATH "" FORCE)
set(Python_NumPy_INCLUDE_DIRS ${DETECTED_NUMPY_INCLUDE_DIR} CACHE STRING "" FORCE)

rosidl_generate_interfaces(${PROJECT_NAME}
	"msg/BmsCmd.msg"
	"msg/BmsState.msg"
	"msg/HandCmd.msg"
	"msg/HandState.msg"
	"msg/IMUState.msg"
	"msg/LowCmd.msg"
	"msg/LowState.msg"
	"msg/MainBoardState.msg"
	"msg/MotorCmd.msg"
	"msg/MotorState.msg"
	"msg/PressSensorState.msg"
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
