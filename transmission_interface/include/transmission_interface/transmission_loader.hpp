// Copyright 2020 ros2_control development team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TRANSMISSION_INTERFACE__TRANSMISSION_LOADER_HPP_
#define TRANSMISSION_INTERFACE__TRANSMISSION_LOADER_HPP_

#include <vector>

#include "hardware_interface/robot_hardware.hpp"
#include "transmission_interface/transmission_info.hpp"
#include "transmission_interface/visibility_control.h"

namespace transmission_interface
{
class TransmissionLoader
{
  public:
    TransmissionLoader(hardware_interface::RobotHardwareSharedPtr robot_hw);
    // , RobotTransmissions& robot_transmissions

    /**
     * /brief Parse transmission information from a URDF
     * /param urdf A string containing the URDF xml
     * /return parsed transmission information
     * /throws std::runtime_error on malformed or empty xml
     */
    bool load(const std::vector<TransmissionInfo>& info);
};

}  // namespace transmission_interface
#endif  // TRANSMISSION_INTERFACE__TRANSMISSION_LOADER_HPP_
