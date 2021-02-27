TEMPLATE = app
CONFIG += c++11 console
CONFIG -= qt

TARGET = CPPSim

INCLUDEPATH += ../src
INCLUDEPATH += ../lib

SOURCES += ../src/*.cpp \
    ../../controls_cpp/build/CMakeFiles/3.19.4/CompilerIdC/CMakeCCompilerId.c \
    ../../controls_cpp/build/CMakeFiles/3.19.4/CompilerIdCXX/CMakeCXXCompilerId.cpp \
    ../../controls_cpp/src/BaseController.cpp \
    ../../controls_cpp/src/Drawing/ColorUtils.cpp \
    ../../controls_cpp/src/Drawing/DrawingFuncs.cpp \
    ../../controls_cpp/src/Drawing/GLUTMenu.cpp \
    ../../controls_cpp/src/Drawing/Graph.cpp \
    ../../controls_cpp/src/Drawing/GraphManager.cpp \
    ../../controls_cpp/src/Drawing/Visualizer_GLUT.cpp \
    ../../controls_cpp/src/Math/Geometry.cpp \
    ../../controls_cpp/src/Math/Random.cpp \
    ../../controls_cpp/src/MavlinkNode/MavlinkNode.cpp \
    ../../controls_cpp/src/MavlinkNode/MavlinkTranslation.cpp \
    ../../controls_cpp/src/MavlinkNode/PracticalSocket.cpp \
    ../../controls_cpp/src/QuadControl.cpp \
    ../../controls_cpp/src/Simulation/BaseDynamics.cpp \
    ../../controls_cpp/src/Simulation/QuadDynamics.cpp \
    ../../controls_cpp/src/Simulation/Simulator.cpp \
    ../../controls_cpp/src/Simulation/magnetometer.cpp \
    ../../controls_cpp/src/Simulation/opticalflow.cpp \
    ../../controls_cpp/src/Simulation/rangefinder.cpp \
    ../../controls_cpp/src/Trajectory.cpp \
    ../../controls_cpp/src/Utility/Camera.cpp \
    ../../controls_cpp/src/Utility/SimpleConfig.cpp \
    ../../controls_cpp/src/Utility/Timer.cpp \
    ../../controls_cpp/src/main.cpp
SOURCES += ../src/Drawing/*.cpp
SOURCES += ../src/Math/*.cpp
SOURCES += ../src/Simulation/*.cpp
SOURCES += ../src/Utility/*.cpp
SOURCES += ../src/MavlinkNode/*.cpp

HEADERS += ../src/*.h \
    ../../controls_cpp/lib/freeglut/include/GL/freeglut.h \
    ../../controls_cpp/lib/freeglut/include/GL/freeglut_ext.h \
    ../../controls_cpp/lib/freeglut/include/GL/freeglut_std.h \
    ../../controls_cpp/lib/freeglut/include/GL/glut.h \
    ../../controls_cpp/lib/matrix/AxisAngle.hpp \
    ../../controls_cpp/lib/matrix/Dcm.hpp \
    ../../controls_cpp/lib/matrix/Euler.hpp \
    ../../controls_cpp/lib/matrix/Matrix.hpp \
    ../../controls_cpp/lib/matrix/Quaternion.hpp \
    ../../controls_cpp/lib/matrix/Scalar.hpp \
    ../../controls_cpp/lib/matrix/SquareMatrix.hpp \
    ../../controls_cpp/lib/matrix/Vector.hpp \
    ../../controls_cpp/lib/matrix/Vector2.hpp \
    ../../controls_cpp/lib/matrix/Vector3.hpp \
    ../../controls_cpp/lib/matrix/filter.hpp \
    ../../controls_cpp/lib/matrix/helper_functions.hpp \
    ../../controls_cpp/lib/matrix/integration.hpp \
    ../../controls_cpp/lib/matrix/math.hpp \
    ../../controls_cpp/lib/mavlink/checksum.h \
    ../../controls_cpp/lib/mavlink/common/common.h \
    ../../controls_cpp/lib/mavlink/common/mavlink.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_actuator_control_target.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_adsb_vehicle.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_altitude.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_att_pos_mocap.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_attitude.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_attitude_quaternion.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_attitude_quaternion_cov.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_attitude_target.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_auth_key.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_autopilot_version.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_battery_status.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_button_change.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_camera_capture_status.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_camera_image_captured.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_camera_information.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_camera_settings.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_camera_trigger.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_change_operator_control.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_change_operator_control_ack.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_collision.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_command_ack.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_command_int.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_command_long.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_control_system_state.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_data_stream.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_data_transmission_handshake.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_debug.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_debug_vect.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_distance_sensor.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_encapsulated_data.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_estimator_status.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_extended_sys_state.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_file_transfer_protocol.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_flight_information.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_follow_target.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_global_position_int.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_global_position_int_cov.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_global_vision_position_estimate.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_gps2_raw.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_gps2_rtk.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_gps_global_origin.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_gps_inject_data.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_gps_input.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_gps_raw_int.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_gps_rtcm_data.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_gps_rtk.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_gps_status.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_heartbeat.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_high_latency.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_high_latency2.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_highres_imu.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_hil_actuator_controls.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_hil_controls.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_hil_gps.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_hil_optical_flow.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_hil_rc_inputs_raw.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_hil_sensor.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_hil_state.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_hil_state_quaternion.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_home_position.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_landing_target.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_local_position_ned.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_local_position_ned_cov.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_local_position_ned_system_global_offset.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_log_data.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_log_entry.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_log_erase.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_log_request_data.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_log_request_end.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_log_request_list.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_logging_ack.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_logging_data.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_logging_data_acked.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_manual_control.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_manual_setpoint.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_memory_vect.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_message_interval.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_mission_ack.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_mission_clear_all.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_mission_count.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_mission_current.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_mission_item.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_mission_item_int.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_mission_item_reached.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_mission_request.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_mission_request_int.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_mission_request_list.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_mission_request_partial_list.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_mission_set_current.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_mission_write_partial_list.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_mount_orientation.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_named_value_float.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_named_value_int.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_nav_controller_output.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_obstacle_distance.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_optical_flow.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_optical_flow_rad.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_param_ext_ack.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_param_ext_request_list.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_param_ext_request_read.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_param_ext_set.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_param_ext_value.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_param_map_rc.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_param_request_list.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_param_request_read.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_param_set.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_param_value.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_ping.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_play_tune.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_position_target_global_int.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_position_target_local_ned.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_power_status.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_protocol_version.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_radio_status.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_raw_imu.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_raw_pressure.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_rc_channels.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_rc_channels_override.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_rc_channels_raw.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_rc_channels_scaled.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_request_data_stream.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_resource_request.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_safety_allowed_area.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_safety_set_allowed_area.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_scaled_imu.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_scaled_imu2.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_scaled_imu3.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_scaled_pressure.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_scaled_pressure2.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_scaled_pressure3.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_serial_control.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_servo_output_raw.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_set_actuator_control_target.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_set_attitude_target.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_set_gps_global_origin.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_set_home_position.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_set_mode.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_set_position_target_global_int.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_set_position_target_local_ned.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_set_video_stream_settings.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_setup_signing.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_sim_state.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_statustext.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_storage_information.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_sys_status.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_system_time.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_terrain_check.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_terrain_data.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_terrain_report.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_terrain_request.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_timesync.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_uavcan_node_info.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_uavcan_node_status.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_v2_extension.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_vfr_hud.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_vibration.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_vicon_position_estimate.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_video_stream_information.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_vision_position_estimate.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_vision_speed_estimate.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_wifi_config_ap.h \
    ../../controls_cpp/lib/mavlink/common/mavlink_msg_wind_cov.h \
    ../../controls_cpp/lib/mavlink/common/testsuite.h \
    ../../controls_cpp/lib/mavlink/common/version.h \
    ../../controls_cpp/lib/mavlink/mavlink_conversions.h \
    ../../controls_cpp/lib/mavlink/mavlink_get_info.h \
    ../../controls_cpp/lib/mavlink/mavlink_helpers.h \
    ../../controls_cpp/lib/mavlink/mavlink_sha256.h \
    ../../controls_cpp/lib/mavlink/mavlink_types.h \
    ../../controls_cpp/lib/mavlink/protocol.h \
    ../../controls_cpp/src/BaseController.h \
    ../../controls_cpp/src/Common.h \
    ../../controls_cpp/src/ControllerFactory.h \
    ../../controls_cpp/src/DataSource.h \
    ../../controls_cpp/src/Drawing/AbsThreshold.h \
    ../../controls_cpp/src/Drawing/BaseAnalyzer.h \
    ../../controls_cpp/src/Drawing/ColorUtils.h \
    ../../controls_cpp/src/Drawing/DrawingFuncs.h \
    ../../controls_cpp/src/Drawing/GLUTMenu.h \
    ../../controls_cpp/src/Drawing/Graph.h \
    ../../controls_cpp/src/Drawing/GraphManager.h \
    ../../controls_cpp/src/Drawing/Visualizer_GLUT.h \
    ../../controls_cpp/src/Drawing/WindowThreshold.h \
    ../../controls_cpp/src/Math/Angles.h \
    ../../controls_cpp/src/Math/Constants.h \
    ../../controls_cpp/src/Math/Geometry.h \
    ../../controls_cpp/src/Math/LowPassFilter.h \
    ../../controls_cpp/src/Math/Mat3x3F.h \
    ../../controls_cpp/src/Math/MathUtils.h \
    ../../controls_cpp/src/Math/Quaternion.h \
    ../../controls_cpp/src/Math/Random.h \
    ../../controls_cpp/src/Math/Transform3D.h \
    ../../controls_cpp/src/Math/V3D.h \
    ../../controls_cpp/src/Math/V3F.h \
    ../../controls_cpp/src/Math/V4D.h \
    ../../controls_cpp/src/MavlinkNode/MavlinkNode.h \
    ../../controls_cpp/src/MavlinkNode/MavlinkTranslation.h \
    ../../controls_cpp/src/MavlinkNode/PracticalSocket.h \
    ../../controls_cpp/src/MavlinkNode/UDPPacket.h \
    ../../controls_cpp/src/QuadControl.h \
    ../../controls_cpp/src/Simulation/BaseDynamics.h \
    ../../controls_cpp/src/Simulation/QuadDynamics.h \
    ../../controls_cpp/src/Simulation/Simulator.h \
    ../../controls_cpp/src/Simulation/magnetometer.h \
    ../../controls_cpp/src/Simulation/opticalflow.h \
    ../../controls_cpp/src/Simulation/rangefinder.h \
    ../../controls_cpp/src/Trajectory.h \
    ../../controls_cpp/src/Utility/Camera.h \
    ../../controls_cpp/src/Utility/FastDelegate.h \
    ../../controls_cpp/src/Utility/FixedQueue.h \
    ../../controls_cpp/src/Utility/Mutex.h \
    ../../controls_cpp/src/Utility/SimpleConfig.h \
    ../../controls_cpp/src/Utility/StringUtils.h \
    ../../controls_cpp/src/Utility/Timer.h \
    ../../controls_cpp/src/VehicleDatatypes.h
HEADERS += ../src/Drawing/*.h
HEADERS += ../src/Math/*.h
HEADERS += ../src/Simulation/*.h
HEADERS += ../src/Utility/*.h
HEADERS += ../src/MavlinkNode/*.h
HEADERS += ../lib/matrix/*.hpp
HEADERS += ../lib/mavlink/*.h
HEADERS += ../lib/mavlink/common/*.h

LIBS += -lglut -lGLU -lGL -lpthread

QMAKE_CXXFLAGS += -Wno-unused-parameter -Wno-unused-local-typedefs

SUBDIRS += \
    ../../controls_cpp/project/CPPSim.pro

DISTFILES += \
    ../../controls_cpp/CMakeLists.txt \
    ../../controls_cpp/README.md \
    ../../controls_cpp/animations/scenario1.gif \
    ../../controls_cpp/animations/scenario2.gif \
    ../../controls_cpp/animations/scenario3.gif \
    ../../controls_cpp/animations/scenario4.gif \
    ../../controls_cpp/animations/scenario5.gif \
    ../../controls_cpp/build-CPPSim-Desktop-Debug/CPPSim \
    ../../controls_cpp/build/CMakeCache.txt \
    ../../controls_cpp/build/CMakeFiles/3.19.4/CMakeCCompiler.cmake \
    ../../controls_cpp/build/CMakeFiles/3.19.4/CMakeCXXCompiler.cmake \
    ../../controls_cpp/build/CMakeFiles/3.19.4/CMakeDetermineCompilerABI_C.bin \
    ../../controls_cpp/build/CMakeFiles/3.19.4/CMakeDetermineCompilerABI_CXX.bin \
    ../../controls_cpp/build/CMakeFiles/3.19.4/CMakeSystem.cmake \
    ../../controls_cpp/build/CMakeFiles/3.19.4/CompilerIdC/a.out \
    ../../controls_cpp/build/CMakeFiles/3.19.4/CompilerIdCXX/a.out \
    ../../controls_cpp/build/CMakeFiles/CMakeDirectoryInformation.cmake \
    ../../controls_cpp/build/CMakeFiles/CMakeOutput.log \
    ../../controls_cpp/build/CMakeFiles/CPPSim.dir/CXX.includecache \
    ../../controls_cpp/build/CMakeFiles/CPPSim.dir/DependInfo.cmake \
    ../../controls_cpp/build/CMakeFiles/CPPSim.dir/build.make \
    ../../controls_cpp/build/CMakeFiles/CPPSim.dir/cmake_clean.cmake \
    ../../controls_cpp/build/CMakeFiles/CPPSim.dir/depend.internal \
    ../../controls_cpp/build/CMakeFiles/CPPSim.dir/depend.make \
    ../../controls_cpp/build/CMakeFiles/CPPSim.dir/flags.make \
    ../../controls_cpp/build/CMakeFiles/CPPSim.dir/link.txt \
    ../../controls_cpp/build/CMakeFiles/CPPSim.dir/progress.make \
    ../../controls_cpp/build/CMakeFiles/TargetDirectories.txt \
    ../../controls_cpp/build/CMakeFiles/cmake.check_cache \
    ../../controls_cpp/build/CMakeFiles/progress.marks \
    ../../controls_cpp/build/CPPSim \
    ../../controls_cpp/build/cmake_install.cmake \
    ../../controls_cpp/config/1_Intro.txt \
    ../../controls_cpp/config/2_AttitudeControl.txt \
    ../../controls_cpp/config/3_PositionControl.txt \
    ../../controls_cpp/config/4_Nonidealities.txt \
    ../../controls_cpp/config/5_TrajectoryFollow.txt \
    ../../controls_cpp/config/LastScenario.txt \
    ../../controls_cpp/config/QuadControlParams.txt \
    ../../controls_cpp/config/QuadPhysicalParams.txt \
    ../../controls_cpp/config/Scenarios.txt \
    ../../controls_cpp/config/Simulation.txt \
    ../../controls_cpp/config/X_Scenarios.txt \
    ../../controls_cpp/config/X_TestManyQuads.txt \
    ../../controls_cpp/config/X_TestMavlink.txt \
    ../../controls_cpp/config/traj/CircleNoFF.txt \
    ../../controls_cpp/config/traj/FigureEight.txt \
    ../../controls_cpp/config/traj/FigureEightFF.txt \
    ../../controls_cpp/config/traj/HelixNoFF.txt \
    ../../controls_cpp/config/traj/HelixUpDownNoFF.txt \
    ../../controls_cpp/config/traj/MakeCircleTrajectory.py \
    ../../controls_cpp/config/traj/MakeHelixTrajectory.py \
    ../../controls_cpp/config/traj/MakeHelixUpDownTrajectory.py \
    ../../controls_cpp/config/traj/MakePeriodicTrajectory.py \
    ../../controls_cpp/config/traj/MakeSpiralTrajectory.py \
    ../../controls_cpp/config/traj/Origin.txt \
    ../../controls_cpp/config/traj/SpiralNoFF.txt \
    ../../controls_cpp/lib/freeglut/Copying.txt \
    ../../controls_cpp/lib/freeglut/Readme.txt \
    ../../controls_cpp/lib/freeglut/bin/freeglut.dll \
    ../../controls_cpp/lib/freeglut/bin/x64/freeglut.dll \
    ../../controls_cpp/lib/freeglut/lib/freeglut.lib \
    ../../controls_cpp/lib/freeglut/lib/x64/freeglut.lib \
    ../../controls_cpp/lib/matrix/LICENSE \
    ../../controls_cpp/notes/Comparing Trajectories.ipynb \
    ../../controls_cpp/notes/FCND-Controls_py/CODEOWNERS \
    ../../controls_cpp/notes/FCND-Controls_py/ControlStructure.png \
    ../../controls_cpp/notes/FCND-Controls_py/NOTES.md \
    ../../controls_cpp/notes/FCND-Controls_py/README.md \
    ../../controls_cpp/notes/FCND-Controls_py/controller.py \
    ../../controls_cpp/notes/FCND-Controls_py/controls_flyer.py \
    ../../controls_cpp/notes/FCND-Controls_py/frame_utils.py \
    ../../controls_cpp/notes/FCND-Controls_py/setup.cfg \
    ../../controls_cpp/notes/FCND-Controls_py/test_trajectory.txt \
    ../../controls_cpp/notes/FCND-Controls_py/unity_drone.py \
    ../../controls_cpp/project/FCND-CPPSim.xcodeproj/project.pbxproj \
    ../../controls_cpp/project/FCND-CPPSim.xcodeproj/project.xcworkspace/contents.xcworkspacedata \
    ../../controls_cpp/project/Simulator.sln \
    ../../controls_cpp/project/Simulator.vcxproj \
    ../../controls_cpp/project/Simulator.vcxproj.filters \
    ../../controls_cpp/x64.png
