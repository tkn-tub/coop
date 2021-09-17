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

from direction import Direction
from message import Message
from state import State


class TestLeaderStates:

    @pytest.mark.parametrize("return_value, expected_state",
                             [(None, State.IDLE),
                              (('foo_id', 12.3), State.VEHICLE_AHEAD)
                              ])
    def test_idle_2_vehicle_ahead(self, request, mocker, return_value, expected_state):
        """
        idle -> vehicle ahead
        """
        leader = request.config.leader
        mocker.patch('util.get_leader', return_value=return_value)
        leader.next_step()
        assert leader.state == expected_state

    @pytest.mark.parametrize("return_value, exp_state",
                             [(tuple(), State.IDLE)])
    def test_vehicle_ahead_2_idle(self, request, mocker, return_value, exp_state):
        """
        vehicle ahead -> idle
        """
        leader = request.config.leader
        leader.set_state(State.VEHICLE_AHEAD)
        mocker.patch('util.get_neighbor', return_value=return_value)
        leader.next_step()
        assert leader.state == exp_state

    @pytest.mark.parametrize("front_vehicle, speed, safety_dist, ot_time, lane_speed, exp_state",
                             [([('test_id', 80.1)], 22.2, 24.9, 45.1, float('inf'), State.VEHICLE_AHEAD),
                              ([('test_id', 80.1)], 22.2, 25.0, 45.1, float('inf'), State.VEHICLE_AHEAD),
                              ([('test_id', 80.1)], 22.2, 24.9, 45.0, float('inf'), State.VEHICLE_AHEAD),
                              ([('test_id', 80.1)], 22.2, 25.0, 44.55, float('inf'), State.ASSERT_OWN_AREAS)
                              ])
    def test_vehicle_ahead_2_assert_own_areas(self, request, mocker, front_vehicle, speed, safety_dist,
                                              ot_time, lane_speed, exp_state):
        """
        vehicle ahead -> assert alpha and beta areas
        """
        leader = request.config.leader
        leader.set_state(State.VEHICLE_AHEAD)
        leader.v_desired = safety_dist
        mocker.patch('util.get_neighbor', return_value=front_vehicle)
        mocker.patch('util.get_speed', return_value=speed)
        mocker.patch('util.get_time', return_value=1)
        mocker.patch('platoon_vehicle.PlatoonVehicle.safety_dist_platoon', return_value=safety_dist)
        mocker.patch('platoon_leader.PlatoonLeader.t_over', return_value=ot_time)
        mocker.patch('platoon_leader.PlatoonLeader.lane_speed', return_value=lane_speed)
        leader.next_step()
        assert leader.state == exp_state

    @pytest.mark.parametrize("alpha_free, beta_free, exp_state",
                             [(False, False, State.LANE_CHANGE_ABORTED),
                              (False, True, State.LANE_CHANGE_ABORTED),
                              (True, False, State.LANE_CHANGE_ABORTED),
                              (True, True, State.REQUEST_SENSOR_DATA)])
    def test_assert_own_areas_2_lane_change_aborted_or_request_sensor_data(self, request, mocker,
                                                                           alpha_free, beta_free, exp_state):
        """
        assert alpha and beta areas -> lane change aborted OR request sensor data
        """
        leader = request.config.leader
        leader.set_state(State.ASSERT_OWN_AREAS)
        mocker.patch('platoon_vehicle.PlatoonVehicle.alpha_area_free', return_value=alpha_free)
        mocker.patch('platoon_vehicle.PlatoonVehicle.beta_area_free', return_value=beta_free)
        leader.next_step()
        assert leader.state == exp_state

    @pytest.mark.parametrize("direction, exp_state", [(Direction.LEFT, State.VEHICLE_AHEAD),
                                                      (Direction.RIGHT, State.PASSING)])
    def test_lane_change_aborted_2_vehicle_ahead_or_passing(self, request, direction, exp_state):
        """
        lane change aborted -> vehicle ahead OR passing
        """
        leader = request.config.leader
        leader.set_state(State.LANE_CHANGE_ABORTED)
        leader.direction = direction
        leader.next_step()
        assert leader.state == exp_state
        assert leader.timer > 0

    def test_request_sensor_data_2_wait_for_responses(self, request):
        """
        request sensor data -> wait for responses
        """
        leader = request.config.leader
        leader.set_state(State.REQUEST_SENSOR_DATA)
        assert len(leader.c2x.queue) == 0
        leader.next_step()
        assert len(leader.c2x.queue) == 1
        assert leader.state == State.WAIT_FOR_RESPONSES

    def test_wait_for_responses_2_lane_change_aborted(self, request):
        """
        wait for responses -> lane change aborted
        """
        leader = request.config.leader
        leader.set_state(State.WAIT_FOR_RESPONSES)
        leader.next_step()
        assert leader.state == State.LANE_CHANGE_ABORTED

    def test_wait_for_responses_2_assert_maneuver_area(self, request, mocker):
        """
        wait for responses -> assert maneuver area
        """
        leader = request.config.leader
        leader.set_state(State.WAIT_FOR_RESPONSES)
        leader.timer = 10
        leader.waiting_for_reply = ['Platoon_1']
        mocker.patch('c2x.C2X.receive', return_value=[{'id': 0,
                                                       'src': 'Platoon_1',
                                                       'dest': 'Platoon_0',
                                                       'msg': Message.RESP_SENSOR_DATA,
                                                       'data': None,
                                                       'receive-min-step': 0
                                                       }])
        leader.next_step()
        assert leader.state == State.ASSERT_MANEUVER_AREA

    @pytest.mark.parametrize("assert_result, exp_state", [([{'own-area-free': True}], State.LANE_CHANGE_SAFE),
                                                          ([{'own-area-free': False}], State.LANE_CHANGE_ABORTED)])
    def test_assert_maneuver_area_2_lane_change_aborted_or_lane_change_safe(self, request, assert_result, exp_state):
        """
        assert maneuver area -> lane change aborted OR lane change safe
        """
        leader = request.config.leader
        leader.set_state(State.ASSERT_MANEUVER_AREA)
        leader.safety_data = assert_result
        leader.next_step()
        assert leader.state == exp_state

    def test_lane_change_safe_2_changing_lane(self, request, mocker):
        """
        lane change safe -> changing lane
        """
        leader = request.config.leader
        leader.set_state(State.LANE_CHANGE_SAFE)
        assert len(leader.c2x.queue) == 0
        mocker.patch('util.get_time', result_value=0)
        leader.next_step()
        assert len(leader.c2x.queue) == 1
        assert leader.state == State.CHANGING_LANE

    def test_changing_lane_2_lane_change_complete(self, request, mocker):
        """
        changing lane -> lane change complete
        """
        leader = request.config.leader
        leader.set_state(State.CHANGING_LANE)
        leader.in_lane_change = True
        leader.in_new_lane = True
        mocker.patch('util.get_lateral_speed', return_value=0)
        mocker.patch('util.get_lateral_offset', return_value=0)
        mocker.patch('util.get_neighbor', return_value=[])
        mocker.patch('util.is_centered', return_value=True)
        leader.next_step()
        assert leader.state == State.LANE_CHANGE_COMPLETE

    @pytest.mark.parametrize("direction, exp_states", [(Direction.LEFT, [State.PASSING, State.PASSING]),
                                                       (Direction.RIGHT, [State.OVERTAKING_COMPLETE, State.IDLE])])
    def test_lane_change_complete_2_passing_or_idle(self, request, direction, exp_states):
        """
        lane change complete -> passing OR idle
        """
        leader = request.config.leader
        leader.set_state(State.LANE_CHANGE_COMPLETE)
        leader.direction = direction
        assert len(leader.c2x.queue) == 0
        leader.next_step()
        assert len(leader.c2x.queue) == 1
        assert leader.state == exp_states[0]
        if direction == Direction.RIGHT:
            leader.next_step()
            assert leader.state == exp_states[1]

    @pytest.mark.parametrize("front_vehicle, speed, safety_dist, ot_time, lane_speed, exp_state",
                             [([('test_id', 80.1)], 22.2, 24.9, 45.1, float('inf'), State.ASSERT_OWN_AREAS),
                              ([('test_id', 80.1)], 22.2, 25.0, 45.1, float('inf'), State.ASSERT_OWN_AREAS),
                              ([('test_id', 80.1)], 22.2, 24.9, 45.0, float('inf'), State.ASSERT_OWN_AREAS),
                              ([('test_id', 80.1)], 22.2, 25.0, 44.55, float('inf'), State.PASSING)
                              ])
    def test_passing_2_assert_own_areas(self, request, mocker, front_vehicle, speed, safety_dist,
                                        ot_time, lane_speed, exp_state):
        """
        vehicle ahead -> assert alpha and beta areas
        """
        leader = request.config.leader
        leader.set_state(State.PASSING)
        leader.v_desired = safety_dist
        leader.direction = Direction.RIGHT
        mocker.patch('util.get_neighbor', return_value=front_vehicle)
        mocker.patch('util.get_speed', return_value=speed)
        mocker.patch('platoon_vehicle.PlatoonVehicle.safety_dist_platoon', return_value=safety_dist)
        mocker.patch('platoon_leader.PlatoonLeader.t_over', return_value=ot_time)
        mocker.patch('platoon_leader.PlatoonLeader.lane_speed', return_value=lane_speed)
        leader.next_step()
        assert leader.state == exp_state

    @pytest.mark.parametrize("alpha_free, beta_free, member_not_following, neighbor, should_overtake, exp_state",
                             [(True, False, False, ('foo_id', 30.0), True, State.ABORT),
                              (False, True, False, ('foo_id', 30.0), True, State.ABORT),
                              (False, False, False, ('foo_id', 30.0), True, State.ABORT),
                              (True, True, False, ('foo_id', 30.0), True, State.CHANGING_LANE),
                              (True, True, True, ('foo_id', 30.0), True, State.ABORT),
                              (True, True, True, ('foo_id', 30.0), False, State.ABORT)])
    def test_changing_lane_2_abort_alpha_beta(self, request, mocker,
                                              alpha_free, beta_free, member_not_following, neighbor, should_overtake,
                                              exp_state):
        """
        changing lane -> abort (due to alpha or beta area not free OR member not following OR slower vehicle changed)
        """
        leader = request.config.leader
        leader.in_lane_change = True
        leader.set_state(State.CHANGING_LANE)
        mocker.patch('platoon_vehicle.PlatoonVehicle.alpha_area_free', return_value=alpha_free)
        mocker.patch('platoon_vehicle.PlatoonVehicle.beta_area_free', return_value=beta_free)
        mocker.patch('platoon_vehicle.PlatoonVehicle.front_or_rear_member_not_following',
                     return_value=member_not_following)
        mocker.patch('util.get_lateral_speed', return_value=0.3)
        mocker.patch('util.get_lateral_offset', return_value=0.4)
        mocker.patch('util.get_neighbor', return_value=[neighbor])
        mocker.patch('platoon_leader.PlatoonLeader.should_overtake', return_value=[should_overtake, 0, 0, 0])
        leader.next_step()
        assert leader.state == exp_state

    @pytest.mark.parametrize("msg, exp_state", [([{'id': 0,
                                                   'src': 'Platoon_1',
                                                   'dest': 'Platoon_0',
                                                   'msg': Message.ABORT,
                                                   'data': None,
                                                   'receive-min-step': 0
                                                   }], State.ABORT),
                                                ([], State.CHANGING_LANE)])
    def test_changing_lane_2_abort_message(self, request, mocker, msg, exp_state):
        """
        changing lane -> abort
        """
        leader = request.config.leader
        leader.in_lane_change = True
        leader.set_state(State.CHANGING_LANE)
        mocker.patch('platoon_vehicle.PlatoonVehicle.get_messages_by_type', return_value=msg)
        mocker.patch('util.get_lateral_speed', return_value=0.3)
        mocker.patch('util.get_lateral_offset', return_value=0.4)
        mocker.patch('platoon_vehicle.PlatoonVehicle.alpha_area_free', return_value=True)
        mocker.patch('platoon_vehicle.PlatoonVehicle.beta_area_free', return_value=True)
        mocker.patch('platoon_vehicle.PlatoonVehicle.front_or_rear_member_not_following', return_value=False)
        mocker.patch('util.get_neighbor', return_value=[])
        leader.next_step()
        assert leader.state == exp_state

    def test_abort_2_changing_back(self, request):
        """
        abort -> changing back
        """
        leader = request.config.leader
        leader.set_state(State.ABORT)
        assert len(leader.c2x.queue) == 0
        leader.next_step()
        assert len(leader.c2x.queue) == 1
        assert leader.state == State.CHANGING_BACK

    def test_changing_back_2_lane_change_aborted(self, request, mocker):
        """
        changing back -> lane change aborted
        """
        leader = request.config.leader
        leader.set_state(State.CHANGING_BACK)
        leader.in_new_lane = True
        mocker.patch('util.is_centered', return_value=True)
        leader.next_step()
        assert leader.state == State.LANE_CHANGE_ABORTED
