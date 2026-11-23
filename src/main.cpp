// Copyright 2026 Dun Yan Ong. Licensed under the Apache License, Version 2.0.
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "mac_registration/mac_registration.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int rc = 0;
  try {
    auto node = std::make_shared<mac_registration::MacRegistrationNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("mac_registration"), "%s", e.what());
    rc = 1;
  }
  rclcpp::shutdown();
  return rc;
}
