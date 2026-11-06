// Copyright 2026 Hypertron ROS 2 contributors

#include "astrall_driver/sdk_functions.hpp"

namespace astrall_driver {

SdkFunctions MakeNativeSdkFunctions() {
  SdkFunctions functions;
  functions.init_ = AstrallSdkInit;
  functions.deinit_ = AstrallSdkDeinit;
  functions.heartbeat_ = AstrallHeartbeat;
  functions.get_system_ = AstrallGetSystemStatus;
  functions.get_power_ = AstrallGetPowerStatus;
  functions.get_sport_ = AstrallGetSportStatus;
  functions.get_device_ = AstrallGetDeviceInfo;
  functions.get_imu_ = AstrallGetImuData;
  functions.subscribe_ = AstrallSubscriptionData;
  functions.auth_ = AstrallAuthControl;
  functions.mode_ = AstrallSportModeControl;
  functions.move_ = AstrallMove;
  functions.light_ = AstrallLightControl;
  functions.send_message_ = AstrallSendMessage;
  return functions;
}

}  // namespace astrall_driver
