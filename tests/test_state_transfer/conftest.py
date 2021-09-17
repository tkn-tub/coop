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
from c2x import C2X
from platoon_follower import PlatoonFollower
from platoon_leader import PlatoonLeader
from state import State


@pytest.fixture(autouse=True)
def before_after(request, mocker):
    # Define leader
    request.config.c2x = C2X(7005)
    request.config.leader = PlatoonLeader('test_leader', request.config.c2x)
    assert request.config.leader.state == State.IDLE
    # Add a follower
    request.config.follower = PlatoonFollower('test_follower', request.config.c2x)
    assert request.config.follower.state == State.IDLE
    request.config.leader.follower.append(request.config.follower.v_id)
    assert request.config.leader.follower == ['test_follower']
    # Deactivate TraCi calls
    mocker.patch('traci.vehicle.getLaneIndex', return_value=0)
    mocker.patch('traci.vehicle.getSpeed', return_value=22.0)
    mocker.patch('traci.vehicle.setLaneChangeMode', return_value=True)
    mocker.patch('traci.vehicle.changeLane', return_value=True)
    # Set parameters to thesis values
    par.MEAN_STEP_DURATION_MSG_DELIVERY = 0
    par.MAX_OVERTAKING_TIME = 45
    par.MIN_OVERTAKING_SPEED_DELTA = 2.7
    # run test
    yield
    # after
    pass
