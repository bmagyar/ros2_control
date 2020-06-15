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

#include "transmission_interface/transmission_loader.hpp"
#include "hardware_interface/robot_hardware.hpp"

namespace transmission_interface
{
using hardware_interface::RobotHardwareSharedPtr;

TransmissionLoader::TransmissionLoader(RobotHardwareSharedPtr robot_hw)
{
}

bool TransmissionLoader::load(const std::vector<TransmissionInfo>& info)
{
    return false;
}

}  // namespace transmission_interface
