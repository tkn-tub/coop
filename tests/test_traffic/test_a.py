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


class TestA:
    """
    Decision phase test cases (please see thesis for details).
    """

    def test_approaching_car(self, request):
        """
        A-1
        """
        request.config.expected = ct.build_expected(request,
                                                    {Key.LANES_VISITED: {'leader': [0, 1, 0],
                                                                         'follower': [0, 1, 0]}
                                                     })
        util.add_vehicle(request.config.sim.plexe, 'approaching_car', 'freeway', 30, 1, 33.3,
                         'Car')
        util.set_speed('approaching_car', 33.3)
        util.add_vehicle(request.config.sim.plexe, 'slow_car', 'freeway',
                         request.config.leader_pos + request.config.slow_car_offset, 0,
                         request.config.slowest_speed, 'Truck')
        # Run simulation
        request.config.results = request.config.sim.run(request.config.platoon)
        # Assert results
        ct.did_overtake(request, 'slow_car')
        ct.did_not_overtake(request, 'approaching_car')
        # Leaders area was occupied
        ct.leaders_area_occupied(request, Direction.LEFT)
        ct.leaders_area_occupied(request, Direction.RIGHT)
        # Followers area was occupied
        ct.followers_area_occupied(request, Direction.LEFT)
        ct.followers_area_occupied(request, Direction.RIGHT)

    def test_neighbor_car(self, request):
        """
        A-2
        """
        request.config.expected = ct.build_expected(request,
                                                    {Key.LANES_VISITED: {'leader': [0, 1, 0],
                                                                         'follower': [0, 1, 0]}
                                                     })
        util.add_vehicle(request.config.sim.plexe, 'neighbor_car', 'freeway',
                         request.config.leader_pos - 60,
                         1, 33.3, 'Car')
        util.set_speed('neighbor_car', 33.3)
        util.add_vehicle(request.config.sim.plexe, 'slow_car', 'freeway',
                         request.config.leader_pos + request.config.slow_car_offset, 0,
                         request.config.slowest_speed, 'Truck')
        # Run simulation
        request.config.results = request.config.sim.run(request.config.platoon)
        # Assert results
        ct.did_overtake(request, 'slow_car')
        ct.did_not_overtake(request, 'neighbor_car')
        ct.leaders_area_occupied(request, Direction.LEFT)
        ct.leaders_area_occupied(request, Direction.RIGHT)
        ct.followers_area_not_occupied(request, Direction.LEFT)
        ct.followers_area_occupied(request, Direction.RIGHT)

    @pytest.mark.parametrize("speed", [25.2, 25.05])
    def test_not_useful_or_not_possible(self, request, speed):
        """
        A-3 + A-4
        """
        request.config.expected = ct.build_expected(request,
                                                    {Key.LANES_VISITED: {'leader': [0],
                                                                         'follower': [0]},
                                                     Key.STATES_VISITED: {'leader': [State.IDLE,
                                                                                     State.VEHICLE_AHEAD],
                                                                          'follower': [State.IDLE]}
                                                     })
        util.add_vehicle(request.config.sim.plexe, 'slow_car', 'freeway',
                         request.config.leader_pos + request.config.slow_car_offset, 0,
                         speed, 'Car')
        util.set_speed('slow_car', speed)
        # Run simulation
        request.config.results = request.config.sim.run(request.config.platoon)
        # Assert results
        ct.did_not_overtake(request, 'slow_car')

    @pytest.mark.parametrize("offset_new_slow_car", [-60, -20])
    def test_new_slow_vehicle(self, request, offset_new_slow_car):
        """
        A-6, A-7
        """
        request.config.expected = ct.build_expected(request,
                                                    {Key.LANES_VISITED: {'leader': [0, 1, 0],
                                                                         'follower': [0, 1, 0]}
                                                     })
        test_commands = {1300: {'cmd': 'change_lane',
                                'direction': -1,
                                'v_id': 'new_slow_car'},
                         2000: {'cmd': 'new_speed',
                                'speed': request.config.slowest_speed,
                                'v_id': 'new_slow_car'}
                         }
        # First car
        not_so_slow_speed = par.SPEED - 1.0
        util.add_vehicle(request.config.sim.plexe, 'not_so_slow_car', 'freeway',
                         request.config.leader_pos + 130, 0, not_so_slow_speed, 'Car')
        util.set_speed('not_so_slow_car', par.SPEED - 2.5)
        # New slow car
        util.add_vehicle(request.config.sim.plexe, 'new_slow_car', 'freeway',
                         request.config.leader_pos + offset_new_slow_car, 1, 33.3, 'Car')
        util.set_speed('new_slow_car', 33.3)
        # Run simulation
        request.config.results = request.config.sim.run(request.config.platoon, test_commands=test_commands)
        # Assert results
        ct.did_overtake(request, 'new_slow_car')
        if offset_new_slow_car == -60:
            ct.did_not_overtake(request, 'not_so_slow_car')
        else:
            ct.did_overtake(request, 'not_so_slow_car')

    def test_overtaken_then_overtaking_both(self, request):
        """
        A-8
        """
        request.config.expected = ct.build_expected(request,
                                                    {Key.LANES_VISITED: {'leader': [0, 1, 0],
                                                                         'follower': [0, 1, 0]}
                                                     })
        test_commands = {2800: {'cmd': 'change_lane',
                                'direction': -1,
                                'v_id': 'neighbor_car'},
                         3250: {'cmd': 'new_speed',
                                'speed': request.config.slowest_speed + 1.0,
                                'v_id': 'neighbor_car'}
                         }
        util.add_vehicle(request.config.sim.plexe, 'neighbor_car', 'freeway',
                         80, 1, 33.3, 'Car', color=(255, 125, 0))
        util.set_speed('neighbor_car', 33.3)
        util.add_vehicle(request.config.sim.plexe, 'slow_car', 'freeway',
                         request.config.leader_pos + request.config.slow_car_offset, 0,
                         request.config.slowest_speed, 'Truck')
        # Run simulation
        request.config.results = request.config.sim.run(request.config.platoon, test_commands=test_commands)
        # Assert results
        ct.did_overtake(request, 'slow_car')
        ct.did_overtake(request, 'neighbor_car')

    @pytest.mark.parametrize("pos_second_blocker", [15, 16])
    def test_gap_big_not_big_enough(self, request, pos_second_blocker):
        """
        A-9 + A-10
        """
        request.config.expected = ct.build_expected(request,
                                                    {Key.LANES_VISITED: {'leader': [0, 1, 0],
                                                                         'follower': [0, 1, 0]}
                                                     })
        # First blocker on overtaking lane
        util.add_vehicle(request.config.sim.plexe, 'first_blocker', 'freeway',
                         145, 1, 30.0, 'NoGapCar', color=(255, 125, 0))
        # util.set_speed('first_blocker', 30.0)
        util.set_lane_change_mode('first_blocker', 0b000000000000)
        # Second blocker on overtaking lane
        util.add_vehicle(request.config.sim.plexe, 'second_blocker', 'freeway',
                         pos_second_blocker, 1, 30.0, 'NoGapCar', color=(255, 180, 0))
        # util.set_speed('second_blocker', 30.0)
        util.set_lane_change_mode('second_blocker', 0b000000000000)
        # Slower vehicle
        util.add_vehicle(request.config.sim.plexe, 'slow_car', 'freeway',
                         request.config.leader_pos + request.config.slow_car_offset, 0,
                         request.config.slowest_speed, 'Truck')
        # Run simulation
        request.config.results = request.config.sim.run(request.config.platoon)
        # Assert results
        ct.did_overtake(request, 'slow_car')
        ct.did_not_overtake(request, 'first_blocker')
        if pos_second_blocker == 15:
            ct.did_overtake(request, 'second_blocker')
        else:
            ct.did_not_overtake(request, 'second_blocker')

    def test_message_failure(self, request):
        """
        A-5
        """
        # Enable message delivery failure
        par.MEAN_STEP_DURATION_MSG_DELIVERY = 7
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
