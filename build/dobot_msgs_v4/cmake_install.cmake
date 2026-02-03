# Install script for directory: /home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/lab-3d/dobot_ws/install/dobot_msgs_v4")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/rosidl_interfaces" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/ament_cmake_index/share/ament_index/resource_index/rosidl_interfaces/dobot_msgs_v4")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/dobot_msgs_v4/dobot_msgs_v4" TYPE DIRECTORY FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_generator_c/dobot_msgs_v4/" REGEX "/[^/]*\\.h$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/environment" TYPE FILE FILES "/opt/ros/humble/lib/python3.10/site-packages/ament_package/template/environment_hook/library_path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/environment" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/ament_cmake_environment_hooks/library_path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_generator_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_generator_c.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_generator_c.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/libdobot_msgs_v4__rosidl_generator_c.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_generator_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_generator_c.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_generator_c.so"
         OLD_RPATH "/opt/ros/humble/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_generator_c.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/dobot_msgs_v4/dobot_msgs_v4" TYPE DIRECTORY FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_typesupport_fastrtps_c/dobot_msgs_v4/" REGEX "/[^/]*\\.cpp$" EXCLUDE)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_fastrtps_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_fastrtps_c.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_fastrtps_c.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/libdobot_msgs_v4__rosidl_typesupport_fastrtps_c.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_fastrtps_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_fastrtps_c.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_fastrtps_c.so"
         OLD_RPATH "/home/lab-3d/dobot_ws/build/dobot_msgs_v4:/opt/ros/humble/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_fastrtps_c.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/dobot_msgs_v4/dobot_msgs_v4" TYPE DIRECTORY FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_generator_cpp/dobot_msgs_v4/" REGEX "/[^/]*\\.hpp$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/dobot_msgs_v4/dobot_msgs_v4" TYPE DIRECTORY FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_typesupport_fastrtps_cpp/dobot_msgs_v4/" REGEX "/[^/]*\\.cpp$" EXCLUDE)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_fastrtps_cpp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_fastrtps_cpp.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_fastrtps_cpp.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/libdobot_msgs_v4__rosidl_typesupport_fastrtps_cpp.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_fastrtps_cpp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_fastrtps_cpp.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_fastrtps_cpp.so"
         OLD_RPATH "/opt/ros/humble/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_fastrtps_cpp.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/dobot_msgs_v4/dobot_msgs_v4" TYPE DIRECTORY FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_typesupport_introspection_c/dobot_msgs_v4/" REGEX "/[^/]*\\.h$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_introspection_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_introspection_c.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_introspection_c.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/libdobot_msgs_v4__rosidl_typesupport_introspection_c.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_introspection_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_introspection_c.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_introspection_c.so"
         OLD_RPATH "/home/lab-3d/dobot_ws/build/dobot_msgs_v4:/opt/ros/humble/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_introspection_c.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_c.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_c.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/libdobot_msgs_v4__rosidl_typesupport_c.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_c.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_c.so"
         OLD_RPATH "/home/lab-3d/dobot_ws/build/dobot_msgs_v4:/opt/ros/humble/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_c.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/dobot_msgs_v4/dobot_msgs_v4" TYPE DIRECTORY FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_typesupport_introspection_cpp/dobot_msgs_v4/" REGEX "/[^/]*\\.hpp$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_introspection_cpp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_introspection_cpp.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_introspection_cpp.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/libdobot_msgs_v4__rosidl_typesupport_introspection_cpp.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_introspection_cpp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_introspection_cpp.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_introspection_cpp.so"
         OLD_RPATH "/opt/ros/humble/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_introspection_cpp.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_cpp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_cpp.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_cpp.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/libdobot_msgs_v4__rosidl_typesupport_cpp.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_cpp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_cpp.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_cpp.so"
         OLD_RPATH "/opt/ros/humble/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_cpp.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/environment" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/ament_cmake_environment_hooks/pythonpath.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/environment" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/ament_cmake_environment_hooks/pythonpath.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/dobot_msgs_v4-0.0.0-py3.10.egg-info" TYPE DIRECTORY FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/ament_cmake_python/dobot_msgs_v4/dobot_msgs_v4.egg-info/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/dobot_msgs_v4" TYPE DIRECTORY FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_generator_py/dobot_msgs_v4/" REGEX "/[^/]*\\.pyc$" EXCLUDE REGEX "/\\_\\_pycache\\_\\_$" EXCLUDE)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(
        COMMAND
        "/usr/bin/python3" "-m" "compileall"
        "/home/lab-3d/dobot_ws/install/dobot_msgs_v4/local/lib/python3.10/dist-packages/dobot_msgs_v4"
      )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/lab-3d/dobot_ws/build/dobot_msgs_v4/dobot_msgs_v4__py/cmake_install.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/dobot_msgs_v4/dobot_msgs_v4_s__rosidl_typesupport_fastrtps_c.cpython-310-x86_64-linux-gnu.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/dobot_msgs_v4/dobot_msgs_v4_s__rosidl_typesupport_fastrtps_c.cpython-310-x86_64-linux-gnu.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/dobot_msgs_v4/dobot_msgs_v4_s__rosidl_typesupport_fastrtps_c.cpython-310-x86_64-linux-gnu.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/dobot_msgs_v4" TYPE SHARED_LIBRARY FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_generator_py/dobot_msgs_v4/dobot_msgs_v4_s__rosidl_typesupport_fastrtps_c.cpython-310-x86_64-linux-gnu.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/dobot_msgs_v4/dobot_msgs_v4_s__rosidl_typesupport_fastrtps_c.cpython-310-x86_64-linux-gnu.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/dobot_msgs_v4/dobot_msgs_v4_s__rosidl_typesupport_fastrtps_c.cpython-310-x86_64-linux-gnu.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/dobot_msgs_v4/dobot_msgs_v4_s__rosidl_typesupport_fastrtps_c.cpython-310-x86_64-linux-gnu.so"
         OLD_RPATH "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_generator_py/dobot_msgs_v4:/home/lab-3d/dobot_ws/build/dobot_msgs_v4:/opt/ros/humble/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/dobot_msgs_v4/dobot_msgs_v4_s__rosidl_typesupport_fastrtps_c.cpython-310-x86_64-linux-gnu.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/dobot_msgs_v4/dobot_msgs_v4_s__rosidl_typesupport_introspection_c.cpython-310-x86_64-linux-gnu.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/dobot_msgs_v4/dobot_msgs_v4_s__rosidl_typesupport_introspection_c.cpython-310-x86_64-linux-gnu.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/dobot_msgs_v4/dobot_msgs_v4_s__rosidl_typesupport_introspection_c.cpython-310-x86_64-linux-gnu.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/dobot_msgs_v4" TYPE SHARED_LIBRARY FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_generator_py/dobot_msgs_v4/dobot_msgs_v4_s__rosidl_typesupport_introspection_c.cpython-310-x86_64-linux-gnu.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/dobot_msgs_v4/dobot_msgs_v4_s__rosidl_typesupport_introspection_c.cpython-310-x86_64-linux-gnu.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/dobot_msgs_v4/dobot_msgs_v4_s__rosidl_typesupport_introspection_c.cpython-310-x86_64-linux-gnu.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/dobot_msgs_v4/dobot_msgs_v4_s__rosidl_typesupport_introspection_c.cpython-310-x86_64-linux-gnu.so"
         OLD_RPATH "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_generator_py/dobot_msgs_v4:/home/lab-3d/dobot_ws/build/dobot_msgs_v4:/opt/ros/humble/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/dobot_msgs_v4/dobot_msgs_v4_s__rosidl_typesupport_introspection_c.cpython-310-x86_64-linux-gnu.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/dobot_msgs_v4/dobot_msgs_v4_s__rosidl_typesupport_c.cpython-310-x86_64-linux-gnu.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/dobot_msgs_v4/dobot_msgs_v4_s__rosidl_typesupport_c.cpython-310-x86_64-linux-gnu.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/dobot_msgs_v4/dobot_msgs_v4_s__rosidl_typesupport_c.cpython-310-x86_64-linux-gnu.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/dobot_msgs_v4" TYPE SHARED_LIBRARY FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_generator_py/dobot_msgs_v4/dobot_msgs_v4_s__rosidl_typesupport_c.cpython-310-x86_64-linux-gnu.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/dobot_msgs_v4/dobot_msgs_v4_s__rosidl_typesupport_c.cpython-310-x86_64-linux-gnu.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/dobot_msgs_v4/dobot_msgs_v4_s__rosidl_typesupport_c.cpython-310-x86_64-linux-gnu.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/dobot_msgs_v4/dobot_msgs_v4_s__rosidl_typesupport_c.cpython-310-x86_64-linux-gnu.so"
         OLD_RPATH "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_generator_py/dobot_msgs_v4:/home/lab-3d/dobot_ws/build/dobot_msgs_v4:/opt/ros/humble/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/dobot_msgs_v4/dobot_msgs_v4_s__rosidl_typesupport_c.cpython-310-x86_64-linux-gnu.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_generator_py.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_generator_py.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_generator_py.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_generator_py/dobot_msgs_v4/libdobot_msgs_v4__rosidl_generator_py.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_generator_py.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_generator_py.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_generator_py.so"
         OLD_RPATH "/home/lab-3d/dobot_ws/build/dobot_msgs_v4:/opt/ros/humble/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_generator_py.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/msg" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/msg/RobotStatus.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/msg" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/msg/ToolVectorActual.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/msg" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/msg/PointArray.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/EnableRobot.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/DisableRobot.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/ClearError.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/SpeedFactor.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/User.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/Tool.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/RobotMode.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/SetPayload.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/DO.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/DOInstant.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/ToolDO.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/ToolDOInstant.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/AO.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/AOInstant.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/AccJ.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/AccL.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/VelJ.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/VelL.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/CP.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/PowerOn.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/RunScript.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/Stop.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/Pause.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/Continue.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/PositiveKin.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/InverseKin.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/SetCollisionLevel.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/GetAngle.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/GetPose.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/EmergencyStop.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/ModbusRTUCreate.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/ModbusCreate.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/ModbusClose.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/GetInBits.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/GetInRegs.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/GetCoils.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/SetCoils.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/GetHoldRegs.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/SetHoldRegs.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/SetSafeSkin.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/MovJ.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/MovL.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/RelJointMovJ.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/MoveJog.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/StopMoveJog.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/DOGroup.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/BrakeControl.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/StartDrag.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/EnableSafeSkin.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/GetStartPose.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/StartPath.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/InverseSolution.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/GetErrorID.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/DI.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/ToolDI.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/AI.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/ToolAI.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/DIGroup.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/StopDrag.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/DragSensivity.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/GetDO.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/GetAO.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/GetDOGroup.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/SetTool485.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/SetSafeWallEnable.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/SetToolPower.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/SetToolMode.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/SetBackDistance.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/SetPostCollisionMode.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/SetUser.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/SetTool.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/CalcUser.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/CalcTool.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/GetInputBool.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/GetInputInt.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/GetInputFloat.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/GetOutputBool.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/GetOutputInt.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/GetOutputFloat.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/SetOutputBool.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/SetOutputInt.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/SetOutputFloat.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/MovLIO.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/MovJIO.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/Arc.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/Circle.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/RelMovJTool.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/RelMovLTool.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/RelMovJUser.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/RelMovLUser.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/GetCurrentCommandId.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/ServoJ.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/ServoP.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/TcpDashboard.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/SendJointState.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/SetFloat.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/CheckJson.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/SendFloat.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/Transform.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/SetStaticTF.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/MojaInvKin.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/msg" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/msg/RobotStatus.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/msg" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/msg/ToolVectorActual.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/msg" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/msg/PointArray.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/EnableRobot.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/EnableRobot_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/EnableRobot_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/DisableRobot.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/DisableRobot_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/DisableRobot_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/ClearError.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/ClearError_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/ClearError_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/SpeedFactor.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/SpeedFactor_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/SpeedFactor_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/User.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/User_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/User_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/Tool.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/Tool_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/Tool_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/RobotMode.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/RobotMode_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/RobotMode_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/SetPayload.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/SetPayload_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/SetPayload_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/DO.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/DO_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/DO_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/DOInstant.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/DOInstant_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/DOInstant_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/ToolDO.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/ToolDO_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/ToolDO_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/ToolDOInstant.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/ToolDOInstant_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/ToolDOInstant_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/AO.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/AO_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/AO_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/AOInstant.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/AOInstant_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/AOInstant_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/AccJ.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/AccJ_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/AccJ_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/AccL.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/AccL_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/AccL_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/VelJ.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/VelJ_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/VelJ_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/VelL.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/VelL_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/VelL_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/CP.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/CP_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/CP_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/PowerOn.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/PowerOn_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/PowerOn_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/RunScript.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/RunScript_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/RunScript_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/Stop.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/Stop_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/Stop_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/Pause.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/Pause_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/Pause_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/Continue.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/Continue_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/Continue_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/PositiveKin.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/PositiveKin_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/PositiveKin_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/InverseKin.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/InverseKin_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/InverseKin_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/SetCollisionLevel.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/SetCollisionLevel_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/SetCollisionLevel_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/GetAngle.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/GetAngle_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/GetAngle_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/GetPose.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/GetPose_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/GetPose_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/EmergencyStop.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/EmergencyStop_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/EmergencyStop_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/ModbusRTUCreate.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/ModbusRTUCreate_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/ModbusRTUCreate_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/ModbusCreate.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/ModbusCreate_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/ModbusCreate_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/ModbusClose.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/ModbusClose_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/ModbusClose_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/GetInBits.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/GetInBits_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/GetInBits_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/GetInRegs.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/GetInRegs_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/GetInRegs_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/GetCoils.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/GetCoils_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/GetCoils_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/SetCoils.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/SetCoils_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/SetCoils_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/GetHoldRegs.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/GetHoldRegs_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/GetHoldRegs_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/SetHoldRegs.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/SetHoldRegs_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/SetHoldRegs_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/SetSafeSkin.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/SetSafeSkin_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/SetSafeSkin_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/MovJ.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/MovJ_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/MovJ_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/MovL.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/MovL_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/MovL_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/RelJointMovJ.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/RelJointMovJ_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/RelJointMovJ_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/MoveJog.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/MoveJog_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/MoveJog_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/StopMoveJog.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/StopMoveJog_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/StopMoveJog_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/DOGroup.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/DOGroup_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/DOGroup_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/BrakeControl.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/BrakeControl_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/BrakeControl_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/StartDrag.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/StartDrag_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/StartDrag_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/EnableSafeSkin.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/EnableSafeSkin_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/EnableSafeSkin_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/GetStartPose.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/GetStartPose_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/GetStartPose_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/StartPath.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/StartPath_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/StartPath_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/InverseSolution.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/InverseSolution_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/InverseSolution_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/GetErrorID.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/GetErrorID_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/GetErrorID_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/DI.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/DI_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/DI_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/ToolDI.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/ToolDI_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/ToolDI_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/AI.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/AI_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/AI_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/ToolAI.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/ToolAI_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/ToolAI_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/DIGroup.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/DIGroup_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/DIGroup_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/StopDrag.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/StopDrag_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/StopDrag_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/DragSensivity.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/DragSensivity_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/DragSensivity_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/GetDO.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/GetDO_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/GetDO_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/GetAO.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/GetAO_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/GetAO_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/GetDOGroup.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/GetDOGroup_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/GetDOGroup_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/SetTool485.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/SetTool485_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/SetTool485_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/SetSafeWallEnable.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/SetSafeWallEnable_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/SetSafeWallEnable_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/SetToolPower.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/SetToolPower_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/SetToolPower_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/SetToolMode.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/SetToolMode_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/SetToolMode_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/SetBackDistance.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/SetBackDistance_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/SetBackDistance_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/SetPostCollisionMode.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/SetPostCollisionMode_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/SetPostCollisionMode_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/SetUser.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/SetUser_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/SetUser_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/SetTool.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/SetTool_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/SetTool_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/CalcUser.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/CalcUser_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/CalcUser_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/CalcTool.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/CalcTool_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/CalcTool_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/GetInputBool.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/GetInputBool_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/GetInputBool_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/GetInputInt.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/GetInputInt_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/GetInputInt_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/GetInputFloat.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/GetInputFloat_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/GetInputFloat_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/GetOutputBool.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/GetOutputBool_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/GetOutputBool_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/GetOutputInt.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/GetOutputInt_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/GetOutputInt_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/GetOutputFloat.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/GetOutputFloat_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/GetOutputFloat_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/SetOutputBool.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/SetOutputBool_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/SetOutputBool_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/SetOutputInt.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/SetOutputInt_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/SetOutputInt_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/SetOutputFloat.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/SetOutputFloat_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/SetOutputFloat_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/MovLIO.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/MovLIO_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/MovLIO_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/MovJIO.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/MovJIO_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/MovJIO_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/Arc.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/Arc_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/Arc_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/Circle.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/Circle_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/Circle_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/RelMovJTool.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/RelMovJTool_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/RelMovJTool_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/RelMovLTool.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/RelMovLTool_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/RelMovLTool_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/RelMovJUser.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/RelMovJUser_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/RelMovJUser_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/RelMovLUser.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/RelMovLUser_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/RelMovLUser_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/GetCurrentCommandId.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/GetCurrentCommandId_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/GetCurrentCommandId_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/ServoJ.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/ServoJ_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/ServoJ_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/ServoP.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/ServoP_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/ServoP_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/TcpDashboard.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/TcpDashboard_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/TcpDashboard_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/SendJointState.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/SendJointState_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/SendJointState_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/SetFloat.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/SetFloat_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/SetFloat_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/CheckJson.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/CheckJson_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/CheckJson_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/SendFloat.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/SendFloat_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/SendFloat_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/Transform.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/Transform_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/Transform_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/SetStaticTF.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/SetStaticTF_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/SetStaticTF_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/MojaInvKin.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/MojaInvKin_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/srv/MojaInvKin_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/dobot_msgs_v4")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/dobot_msgs_v4")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/environment" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/environment" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/ament_cmake_environment_hooks/path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/ament_cmake_environment_hooks/local_setup.bash")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/ament_cmake_environment_hooks/local_setup.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/ament_cmake_environment_hooks/package.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/ament_cmake_index/share/ament_index/resource_index/packages/dobot_msgs_v4")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/export_dobot_msgs_v4__rosidl_generator_cExport.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/export_dobot_msgs_v4__rosidl_generator_cExport.cmake"
         "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/CMakeFiles/Export/share/dobot_msgs_v4/cmake/export_dobot_msgs_v4__rosidl_generator_cExport.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/export_dobot_msgs_v4__rosidl_generator_cExport-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/export_dobot_msgs_v4__rosidl_generator_cExport.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/CMakeFiles/Export/share/dobot_msgs_v4/cmake/export_dobot_msgs_v4__rosidl_generator_cExport.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/CMakeFiles/Export/share/dobot_msgs_v4/cmake/export_dobot_msgs_v4__rosidl_generator_cExport-noconfig.cmake")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/export_dobot_msgs_v4__rosidl_typesupport_fastrtps_cExport.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/export_dobot_msgs_v4__rosidl_typesupport_fastrtps_cExport.cmake"
         "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/CMakeFiles/Export/share/dobot_msgs_v4/cmake/export_dobot_msgs_v4__rosidl_typesupport_fastrtps_cExport.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/export_dobot_msgs_v4__rosidl_typesupport_fastrtps_cExport-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/export_dobot_msgs_v4__rosidl_typesupport_fastrtps_cExport.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/CMakeFiles/Export/share/dobot_msgs_v4/cmake/export_dobot_msgs_v4__rosidl_typesupport_fastrtps_cExport.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/CMakeFiles/Export/share/dobot_msgs_v4/cmake/export_dobot_msgs_v4__rosidl_typesupport_fastrtps_cExport-noconfig.cmake")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/export_dobot_msgs_v4__rosidl_generator_cppExport.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/export_dobot_msgs_v4__rosidl_generator_cppExport.cmake"
         "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/CMakeFiles/Export/share/dobot_msgs_v4/cmake/export_dobot_msgs_v4__rosidl_generator_cppExport.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/export_dobot_msgs_v4__rosidl_generator_cppExport-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/export_dobot_msgs_v4__rosidl_generator_cppExport.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/CMakeFiles/Export/share/dobot_msgs_v4/cmake/export_dobot_msgs_v4__rosidl_generator_cppExport.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/export_dobot_msgs_v4__rosidl_typesupport_fastrtps_cppExport.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/export_dobot_msgs_v4__rosidl_typesupport_fastrtps_cppExport.cmake"
         "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/CMakeFiles/Export/share/dobot_msgs_v4/cmake/export_dobot_msgs_v4__rosidl_typesupport_fastrtps_cppExport.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/export_dobot_msgs_v4__rosidl_typesupport_fastrtps_cppExport-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/export_dobot_msgs_v4__rosidl_typesupport_fastrtps_cppExport.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/CMakeFiles/Export/share/dobot_msgs_v4/cmake/export_dobot_msgs_v4__rosidl_typesupport_fastrtps_cppExport.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/CMakeFiles/Export/share/dobot_msgs_v4/cmake/export_dobot_msgs_v4__rosidl_typesupport_fastrtps_cppExport-noconfig.cmake")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/dobot_msgs_v4__rosidl_typesupport_introspection_cExport.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/dobot_msgs_v4__rosidl_typesupport_introspection_cExport.cmake"
         "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/CMakeFiles/Export/share/dobot_msgs_v4/cmake/dobot_msgs_v4__rosidl_typesupport_introspection_cExport.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/dobot_msgs_v4__rosidl_typesupport_introspection_cExport-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/dobot_msgs_v4__rosidl_typesupport_introspection_cExport.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/CMakeFiles/Export/share/dobot_msgs_v4/cmake/dobot_msgs_v4__rosidl_typesupport_introspection_cExport.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/CMakeFiles/Export/share/dobot_msgs_v4/cmake/dobot_msgs_v4__rosidl_typesupport_introspection_cExport-noconfig.cmake")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/dobot_msgs_v4__rosidl_typesupport_cExport.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/dobot_msgs_v4__rosidl_typesupport_cExport.cmake"
         "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/CMakeFiles/Export/share/dobot_msgs_v4/cmake/dobot_msgs_v4__rosidl_typesupport_cExport.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/dobot_msgs_v4__rosidl_typesupport_cExport-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/dobot_msgs_v4__rosidl_typesupport_cExport.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/CMakeFiles/Export/share/dobot_msgs_v4/cmake/dobot_msgs_v4__rosidl_typesupport_cExport.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/CMakeFiles/Export/share/dobot_msgs_v4/cmake/dobot_msgs_v4__rosidl_typesupport_cExport-noconfig.cmake")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/dobot_msgs_v4__rosidl_typesupport_introspection_cppExport.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/dobot_msgs_v4__rosidl_typesupport_introspection_cppExport.cmake"
         "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/CMakeFiles/Export/share/dobot_msgs_v4/cmake/dobot_msgs_v4__rosidl_typesupport_introspection_cppExport.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/dobot_msgs_v4__rosidl_typesupport_introspection_cppExport-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/dobot_msgs_v4__rosidl_typesupport_introspection_cppExport.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/CMakeFiles/Export/share/dobot_msgs_v4/cmake/dobot_msgs_v4__rosidl_typesupport_introspection_cppExport.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/CMakeFiles/Export/share/dobot_msgs_v4/cmake/dobot_msgs_v4__rosidl_typesupport_introspection_cppExport-noconfig.cmake")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/dobot_msgs_v4__rosidl_typesupport_cppExport.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/dobot_msgs_v4__rosidl_typesupport_cppExport.cmake"
         "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/CMakeFiles/Export/share/dobot_msgs_v4/cmake/dobot_msgs_v4__rosidl_typesupport_cppExport.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/dobot_msgs_v4__rosidl_typesupport_cppExport-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/dobot_msgs_v4__rosidl_typesupport_cppExport.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/CMakeFiles/Export/share/dobot_msgs_v4/cmake/dobot_msgs_v4__rosidl_typesupport_cppExport.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/CMakeFiles/Export/share/dobot_msgs_v4/cmake/dobot_msgs_v4__rosidl_typesupport_cppExport-noconfig.cmake")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/export_dobot_msgs_v4__rosidl_generator_pyExport.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/export_dobot_msgs_v4__rosidl_generator_pyExport.cmake"
         "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/CMakeFiles/Export/share/dobot_msgs_v4/cmake/export_dobot_msgs_v4__rosidl_generator_pyExport.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/export_dobot_msgs_v4__rosidl_generator_pyExport-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/export_dobot_msgs_v4__rosidl_generator_pyExport.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/CMakeFiles/Export/share/dobot_msgs_v4/cmake/export_dobot_msgs_v4__rosidl_generator_pyExport.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/CMakeFiles/Export/share/dobot_msgs_v4/cmake/export_dobot_msgs_v4__rosidl_generator_pyExport-noconfig.cmake")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/rosidl_cmake-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/ament_cmake_export_dependencies/ament_cmake_export_dependencies-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/ament_cmake_export_include_directories/ament_cmake_export_include_directories-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/ament_cmake_export_libraries/ament_cmake_export_libraries-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/ament_cmake_export_targets/ament_cmake_export_targets-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/rosidl_cmake_export_typesupport_targets-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake" TYPE FILE FILES "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/rosidl_cmake_export_typesupport_libraries-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake" TYPE FILE FILES
    "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/ament_cmake_core/dobot_msgs_v4Config.cmake"
    "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/ament_cmake_core/dobot_msgs_v4Config-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4" TYPE FILE FILES "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/lab-3d/dobot_ws/build/dobot_msgs_v4/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
