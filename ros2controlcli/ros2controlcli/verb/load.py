# Copyright 2020 PAL Robotics S.L.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys

from controller_manager import load_controller

from ros2cli.node.direct import add_arguments
from ros2cli.node.strategy import NodeStrategy
from ros2cli.verb import VerbExtension

from ros2controlcli.api import add_controller_mgr_parsers, ControllerNameCompleter


class LoadVerb(VerbExtension):
    """Load a controller in a controller manager."""

    def add_arguments(self, parser, cli_name):
        add_arguments(parser)
        arg = parser.add_argument(
            'controller_name', help='Name of the controller')
        arg.completer = ControllerNameCompleter()
        add_controller_mgr_parsers(parser)

    def main(self, *, args):
        with NodeStrategy(args) as node:
            response = load_controller(node, args.controller_manager, args.controller_name)
            if not response.ok:
                print('Error loading controller, check controller_manager logs', file=sys.stderr)
            return not response.ok
