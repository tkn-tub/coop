# Copyright (c) 2021 Martin Strunz <strunz@campus.tu-berlin.de>
# Copyright (c) 2021 Julian Heinovski <heinovski@ccs-labs.org>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

import pytest

import parameter as par
import util
from direction import Direction
from key import Key
from state import State
from tests.test_traffic import conftest as ct


class TestB:
    """
    Decision phase test cases (please see thesis for details).
    """

    @pytest.mark.parametrize("step_remove, lanes_visited", [(350, [0]), (600, [0, 1, 0])])
    def test_slower_vehicle_gone(self, request, step_remove, lanes_visited):
        """
        B-1 + B-2
        """
        par.DISTANCE_TO_DRIVE = 500
        request.config.expected = ct.build_expected(request,
                                                    {Key.LANES_VISITED: {'leader': lanes_visited,
                                                                         'follower': lanes_visited}
                                                     })
        test_commands = {step_remove: {'cmd': 'remove_vehicle',
                                       'v_id': 'slower_vehicle'}}
        util.add_vehicle(request.config.sim.plexe, 'slower_vehicle', 'freeway',
                         request.config.leader_pos + request.config.slow_car_offset - 40, 0,
                         request.config.slowest_speed, 'Truck')
        # Run simulation
        request.config.results = request.config.sim.run(request.config.platoon, test_commands=test_commands)
        # Assert results
        ct.did_visit_state(request, State.CHANGING_BACK, leader=True, follower=True)
        ct.did_not_visit_state(request, State.PASSING, leader=True)

    @pytest.mark.parametrize("step_accel, lanes_visited",
                             [(150, [0]),
                              (300, [0, 1, 0])])
    def test_slower_vehicle_accelerates(self, request, step_accel, lanes_visited):
        """
        B-3 + B-4
        """
        par.DISTANCE_TO_DRIVE = 500
        request.config.expected = ct.build_expected(request,
                                                    {Key.LANES_VISITED: {'leader': lanes_visited,
                                                                         'follower': lanes_visited}
                                                     })
        test_commands = {step_accel: {'cmd': 'new_speed',
                                      'speed': 33.3,
                                      'v_id': 'slower_vehicle'},
                         step_accel + 1: {'cmd': 'new_color',
                                          'color': (47, 86, 233),
                                          'v_id': 'slower_vehicle'}}
        util.add_vehicle(request.config.sim.plexe, 'slower_vehicle', 'freeway',
                         request.config.leader_pos + request.config.slow_car_offset - 80, 0,
                         request.config.slowest_speed, 'Car')
        util.set_speed('slower_vehicle', request.config.slowest_speed)
        util.set_color('slower_vehicle', (255, 0, 0))
        # Run simulation
        request.config.results = request.config.sim.run(request.config.platoon, test_commands=test_commands)
        # Assert results
        ct.did_visit_state(request, State.CHANGING_BACK, leader=True, follower=True)
        ct.did_not_visit_state(request, State.PASSING, leader=True)

    @pytest.mark.parametrize("distance, disruptor_step, disruptor_lane, disruptor_pos, disruptor_speed, pos_type, "
                             "lanes_visited, test_case",
                             [(1750, 4600, 'freeway_0', 1490, par.SPEED - 8, 'abs', [0, 1], 2),  # B-5
                              (1900, 4700, 'freeway_0', 1550, par.SPEED - 8, 'abs', [0, 1, 0, 1], 2),  # B-6
                              (750, 450, 'freeway_1', 220, par.SPEED, 'abs', [0], 1),  # B-7
                              (700, 580, 'freeway_1', 249, par.SPEED, 'abs', [0, 1], 4),  # B-8
                              (230, 450, 'freeway_1', 335, par.SPEED - 8, 'abs', [0], 1),  # B-9
                              (400, 400, 'freeway_1', 275, par.SPEED, 'abs', [0], 1),  # B-10
                              (1480, 4600, 'freeway_0', 1400, par.SPEED - 8, 'abs', [0, 1], 2),  # B-11
                              (1600, 4600, 'freeway_0', 1550, par.SPEED - 8, 'abs', [0, 1], 3),  # B-12
                              (1600, 4520, 'freeway_0', 1420, par.SPEED - 8, 'abs', [0, 1], 2),  # B-13
                              ])
    def test_disruptor_rear_front_side(self, request, distance, disruptor_step, disruptor_lane, disruptor_pos,
                                       disruptor_speed, pos_type, lanes_visited, test_case):
        """
        B-5 - B-13
        """
        par.DISTANCE_TO_DRIVE = distance
        request.config.expected = ct.build_expected(request,
                                                    {Key.LANES_VISITED: {'leader': lanes_visited,
                                                                         'follower': lanes_visited}
                                                     })
        test_commands = {disruptor_step: {'cmd': 'add_vehicle',
                                          'pos': 0,
                                          'speed': disruptor_speed,
                                          'lane_id': disruptor_lane,
                                          'vtype': 'Car',
                                          'v_id': 'disruptor'}}
        if pos_type == 'abs':
            test_commands[disruptor_step]['pos'] = disruptor_pos
        else:
            test_commands[disruptor_step]['pos'] = request.config.leader_pos + disruptor_pos
        util.add_vehicle(request.config.sim.plexe, 'slow_car', 'freeway',
                         request.config.leader_pos + request.config.slow_car_offset - 40, 0,
                         request.config.slowest_speed, 'Truck')
        # Run simulation
        request.config.results = request.config.sim.run(request.config.platoon, test_commands=test_commands)
        # Assert results for left area
        if test_case == 1:
            ct.did_visit_state(request, State.CHANGING_BACK, leader=True, follower=True)
            ct.did_not_overtake(request, 'slow_car')
            ct.did_not_overtake(request, 'disruptor')
        elif test_case == 2:
            ct.did_visit_state(request, State.R_CHANGING_BACK, leader=True, follower=True)
            ct.did_overtake(request, 'slow_car')
            ct.did_overtake(request, 'disruptor')
        elif test_case == 3:
            ct.did_visit_state(request, State.R_CHANGING_BACK, leader=True, follower=True)
            ct.did_overtake(request, 'slow_car')
            ct.did_not_overtake(request, 'disruptor')
        elif test_case == 4:
            ct.did_not_visit_state(request, State.CHANGING_BACK, leader=True, follower=True)
            ct.did_not_overtake(request, 'slow_car')
            ct.did_overtake(request, 'disruptor')

    def test_message_failure(self, request):
        """
        B-14
        """
        # Enable message delivery failure
        par.MEAN_STEP_DURATION_MSG_DELIVERY = 7
        par.DISTANCE_TO_DRIVE = 3000
        request.config.expected = ct.build_expected(request,
                                                    {Key.LANES_VISITED: {'leader': [0, 1, 0],
                                                                         'follower': [0, 1, 0]}
                                                     })
        util.add_vehicle(request.config.sim.plexe, 'slow_car', 'freeway',
                         request.config.leader_pos + request.config.slow_car_offset, 0,
                         request.config.slowest_speed, 'Truck')
        # Run simulation
        request.config.results = request.config.sim.run(request.config.platoon)
        # Assert results
        ct.did_overtake(request, 'slow_car')
        # Leader runs in a timeout
        ct.leader_timeout_in_decision_phase(request, Direction.LEFT)
        ct.leader_timeout_in_decision_phase(request, Direction.RIGHT)
        # Follower do not always get a decision
        ct.decision_not_always_received(request, Direction.LEFT)
        ct.decision_not_always_received(request, Direction.RIGHT)
        # Disable message delivery failure
        par.MEAN_STEP_DURATION_MSG_DELIVERY = 0
