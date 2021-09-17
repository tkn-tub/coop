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

import parameter as par
import util
from direction import Direction
from key import Key
from state import State
from tests.test_traffic import conftest as ct


class TestBasic:

    def test_best_case(self, request):
        """
        Best case: the platoon is not delayed by a slower vehicle.
        """
        request.config.expected = ct.build_expected(request,
                                                    {Key.STATES_VISITED: {'leader': [State.IDLE],
                                                                          'follower': [State.IDLE]},
                                                     Key.LANES_VISITED: {'leader': [0],
                                                                         'follower': [0]},
                                                     Key.SPEED_MIN: {'leader': par.SPEED,
                                                                     'follower': par.SPEED},
                                                     Key.SPEED_MAX: {'leader': par.SPEED,
                                                                     'follower': par.SPEED},
                                                     })
        # Run simulation
        request.config.results = request.config.sim.run(request.config.platoon)

    def test_worst_case(self, mocker, request):
        """
        Worst case: the platoon is delayed by a slower vehicle and does not overtake because the algorithm
        ignores the slower vehicle (ACC is still operating).
        """
        request.config.expected = ct.build_expected(request,
                                                    {Key.STATES_VISITED: {'leader': [State.IDLE],
                                                                          'follower': [State.IDLE]},
                                                     Key.LANES_VISITED: {'leader': [0],
                                                                         'follower': [0]},
                                                     Key.SPEED_MIN: {'leader': request.config.slowest_speed,
                                                                     'follower': request.config.slowest_speed},
                                                     Key.SPEED_MAX: {'leader': par.SPEED,
                                                                     'follower': par.SPEED},
                                                     })
        util.add_vehicle(request.config.sim.plexe, 'slow_car', 'freeway',
                         request.config.leader_pos + request.config.slow_car_offset, 0,
                         request.config.slowest_speed, 'Truck')
        mocker.patch('util.get_leader', return_value=None)
        # Run simulation
        request.config.results = request.config.sim.run(request.config.platoon)
        # Assert results
        ct.did_not_overtake(request, 'slow_car')

    def test_plain_overtaking(self, request):
        """
        Plain overtaking: the platoon detects a slower vehicle and overtakes it without being disturbed by
        other traffic.
        """
        request.config.expected = ct.build_expected(request,
                                                    {Key.LANES_VISITED: {'leader': [0, 1, 0],
                                                                         'follower': [0, 1, 0]},
                                                     Key.SPEED_MIN: {'leader': par.SPEED,
                                                                     'follower': par.SPEED},
                                                     Key.SPEED_MAX: {'leader': par.SPEED,
                                                                     'follower': par.SPEED},
                                                     })
        util.add_vehicle(request.config.sim.plexe, 'slow_car', 'freeway',
                         request.config.leader_pos + request.config.slow_car_offset, 0,
                         request.config.slowest_speed, 'Truck')
        # Run simulation
        request.config.results = request.config.sim.run(request.config.platoon)
        # Assert results
        ct.did_overtake(request, 'slow_car')
        ct.leaders_area_not_occupied(request, Direction.LEFT)
        ct.followers_area_not_occupied(request, Direction.LEFT)
        ct.no_leader_timeout_in_decision_phase(request, Direction.LEFT)
        ct.no_leader_timeout_in_decision_phase(request, Direction.RIGHT)
        ct.decision_always_received(request, Direction.LEFT)
        ct.decision_not_always_received(request, Direction.RIGHT)
        ct.did_not_visit_state(request, State.ABORT, leader=True, follower=True)
        ct.did_not_visit_state(request, State.R_ABORT, leader=True, follower=True)
