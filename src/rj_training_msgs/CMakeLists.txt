cmake_minimum_required(VERSION 3.8)
project(rj_training_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
msg/FortuneCookie.msg
DEPENDENCIES
# As we are using the standard types available, hence dependencies list is empty
ADD_LINTER_TESTS
# Adds various styles checking tests for our interface files
)
#The generate inferface function will create cpp and python code for the interface types we give it

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
# This exports the dependency from this package to the default runtime
# This ensures that anyother package that depends on our package will also indirectly pull in
# the default runtime package.
ament_package()
