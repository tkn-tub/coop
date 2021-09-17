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


class TestFollowerStates:

    @pytest.mark.parametrize("msg_queue, exp_state", [([{'id': 0,
                                                         'src': 'Platoon_0',
                                                         'dest': 'Platoon_1',
                                                         'msg': Message.REQ_SENSOR_DATA,
                                                         'data': {'direction': Direction.LEFT},
                                                         'receive-min-step': 0
                                                         }], State.ASSERT_OWN_AREAS),
                                                      ([], State.IDLE)])
    def test_idle_2_assert_own_areas(self, request, mocker, msg_queue, exp_state):
        """
        idle -> assert alpha and beta areas
        """
        follower = request.config.follower
        mocker.patch('platoon_follower.PlatoonFollower.get_messages_by_type', return_value=msg_queue)
        follower.next_step()
        assert follower.state == exp_state

    @pytest.mark.parametrize("alpha_free, beta_free, exp_state",
                             [(False, False, State.WAIT_FOR_DECISION),
                              (False, True, State.WAIT_FOR_DECISION),
                              (True, False, State.WAIT_FOR_DECISION),
                              (True, True, State.WAIT_FOR_DECISION)])
    def test_assert_own_areas_2_lane_change_aborted_or_request_sensor_data(self, request, mocker,
                                                                           alpha_free, beta_free, exp_state):
        """
        assert alpha and beta areas -> wait for decision
        """
        follower = request.config.follower
        follower.set_state(State.ASSERT_OWN_AREAS)
        mocker.patch('platoon_vehicle.PlatoonVehicle.alpha_area_free', return_value=alpha_free)
        mocker.patch('platoon_vehicle.PlatoonVehicle.beta_area_free', return_value=beta_free)
        assert len(follower.c2x.queue) == 0
        follower.next_step()
        assert len(follower.c2x.queue) == 1
        assert follower.timer > 0
        assert follower.state == exp_state

    def test_wait_for_decision_2_idle(self, request):
        """
        wait for decision -> idle
        """
        follower = request.config.follower
        follower.set_state(State.WAIT_FOR_DECISION)
        follower.timer = 0
        follower.next_step()
        assert follower.state == State.IDLE

    @pytest.mark.parametrize("msg_queue, exp_state", [([{'id': 0,
                                                         'src': 'Platoon_0',
                                                         'dest': 'Platoon_1',
                                                         'msg': Message.BEGIN_LANE_CHANGE,
                                                         'data': {'direction': Direction.LEFT},
                                                         'receive-min-step': 0
                                                         }], State.CHANGING_LANE),
                                                      ([], State.WAIT_FOR_DECISION)])
    def test_wait_for_decision_2_changing_lane(self, request, mocker, msg_queue, exp_state):
        """
        wait for decision -> changing lane
        """
        follower = request.config.follower
        follower.set_state(State.WAIT_FOR_DECISION)
        follower.timer = 10
        mocker.patch('platoon_follower.PlatoonFollower.get_messages_by_type', return_value=msg_queue)
        follower.next_step()
        assert follower.state == exp_state

    def test_changing_lane_2_lane_changed(self, request, mocker):
        """
        changing lane -> lane changed
        """
        follower = request.config.follower
        follower.set_state(State.CHANGING_LANE)
        follower.in_lane_change = True
        follower.in_new_lane = True
        mocker.patch('util.get_lateral_speed', return_value=0)
        mocker.patch('util.get_lateral_offset', return_value=0)
        mocker.patch('util.get_neighbor', return_value=[])
        mocker.patch('util.is_centered', return_value=True)
        follower.next_step()
        assert follower.state == State.LANE_CHANGED

    @pytest.mark.parametrize("msg_queue, exp_state", [(([{'id': 0,
                                                          'src': 'Platoon_0',
                                                          'dest': 'Platoon_1',
                                                          'msg': Message.ABORT,
                                                          'data': {'direction': Direction.LEFT},
                                                          'receive-min-step': 0
                                                          }], []), State.CHANGING_BACK),
                                                      (([], []), State.IN_OVERTAKING_LANE)])
    def test_lane_changed_2_abort_or_in_overtaking_lane(self, request, mocker, msg_queue, exp_state):
        """
        lane changed -> abort OR in overtaking lane (not a separate FSM state, belongs to lane changed)
        """
        follower = request.config.follower
        follower.set_state(State.LANE_CHANGED)
        mocker.patch('platoon_follower.PlatoonFollower.get_messages_by_type', side_effect=msg_queue)
        follower.next_step()
        assert follower.state == exp_state

    @pytest.mark.parametrize("msg_queue, exp_state", [([{'id': 0,
                                                         'src': 'Platoon_0',
                                                         'dest': 'Platoon_1',
                                                         'msg': Message.LANE_CHANGE_COMPLETE,
                                                         'data': {'direction': Direction.LEFT},
                                                         'receive-min-step': 0
                                                         }], State.LANE_CHANGE_COMPLETE),
                                                      ([], State.IN_OVERTAKING_LANE)])
    def test_in_overtaking_lane_2_idle(self, request, mocker, msg_queue, exp_state):
        """
        in overtaking lane -> idle (via the 'clean-up' state overtaking complete)
        """
        follower = request.config.follower
        follower.set_state(State.IN_OVERTAKING_LANE)
        mocker.patch('platoon_follower.PlatoonFollower.get_messages_by_type', return_value=msg_queue)
        follower.next_step()
        assert follower.state == exp_state
        if exp_state == State.LANE_CHANGE_COMPLETE:
            follower.next_step()
            assert follower.state == State.IDLE

    @pytest.mark.parametrize("alpha_free, beta_free, member_not_following, neighbor, exp_state",
                             [(True, False, False, ('foo_id', 30.0), State.ABORT),
                              (False, True, False, ('foo_id', 30.0), State.ABORT),
                              (False, False, False, ('foo_id', 30.0), State.ABORT),
                              (True, True, False, ('foo_id', 30.0), State.CHANGING_LANE),
                              (True, True, True, ('foo_id', 30.0), State.ABORT)])
    def test_changing_lane_2_abort_alpha_beta(self, request, mocker,
                                              alpha_free, beta_free, member_not_following, neighbor, exp_state):
        """
        changing lane -> abort (due to alpha or beta area not free OR member not following)
        """
        follower = request.config.follower
        follower.in_lane_change = True
        follower.set_state(State.CHANGING_LANE)
        mocker.patch('platoon_vehicle.PlatoonVehicle.alpha_area_free', return_value=alpha_free)
        mocker.patch('platoon_vehicle.PlatoonVehicle.beta_area_free', return_value=beta_free)
        mocker.patch('platoon_vehicle.PlatoonVehicle.front_or_rear_member_not_following',
                     return_value=member_not_following)
        mocker.patch('util.get_lateral_speed', return_value=0.3)
        mocker.patch('util.get_lateral_offset', return_value=0.4)
        mocker.patch('util.get_neighbor', return_value=[neighbor])
        follower.next_step()
        assert follower.state == exp_state

    @pytest.mark.parametrize("msg, exp_state", [([{'id': 0,
                                                   'src': 'Platoon_0',
                                                   'dest': 'Platoon_1',
                                                   'msg': Message.ABORT,
                                                   'data': None,
                                                   'receive-min-step': 0
                                                   }], State.CHANGING_BACK),
                                                ([], State.CHANGING_LANE)])
    def test_changing_lane_2_changing_back(self, request, mocker, msg, exp_state):
        """
        changing lane -> changing back
        """
        follower = request.config.follower
        follower.in_lane_change = True
        follower.set_state(State.CHANGING_LANE)
        mocker.patch('platoon_vehicle.PlatoonVehicle.get_messages_by_type', return_value=msg)
        mocker.patch('util.get_lateral_speed', return_value=0.3)
        mocker.patch('util.get_lateral_offset', return_value=0.4)
        mocker.patch('platoon_vehicle.PlatoonVehicle.alpha_area_free', return_value=True)
        mocker.patch('platoon_vehicle.PlatoonVehicle.beta_area_free', return_value=True)
        mocker.patch('platoon_vehicle.PlatoonVehicle.front_or_rear_member_not_following', return_value=False)
        mocker.patch('util.get_neighbor', return_value=[])
        follower.next_step()
        assert follower.state == exp_state

    def test_abort_2_changing_back(self, request):
        """
        abort -> changing back
        """
        follower = request.config.follower
        follower.set_state(State.ABORT)
        assert len(follower.c2x.queue) == 0
        follower.next_step()
        assert len(follower.c2x.queue) == 1
        assert follower.state == State.CHANGING_BACK

    def test_changing_back_2_in_original_lane(self, request, mocker):
        """
        changing back -> in original lane
        """
        follower = request.config.follower
        follower.set_state(State.CHANGING_BACK)
        follower.in_new_lane = True
        mocker.patch('util.is_centered', return_value=True)
        follower.next_step()
        assert follower.state == State.IN_ORIGINAL_LANE

    def test_in_original_lane_2_idle(self, request):
        """
        in original lane -> idle
        """
        follower = request.config.follower
        follower.set_state(State.IN_ORIGINAL_LANE)
        assert len(follower.c2x.queue) == 0
        follower.next_step()
        assert len(follower.c2x.queue) == 1
        assert follower.state == State.IDLE
