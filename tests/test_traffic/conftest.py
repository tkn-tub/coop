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
from direction import Direction
from key import Key
from platoon import Platoon
from simulation import Simulation
from state import State


@pytest.fixture(autouse=True, params=[4])
def before_after(request):
    """
    Instructions for all test cases.
    """
    par.EXIT_ON_OVERTAKING = True
    par.SPEED = 27.8
    # Speed of slowest vehicle in the simulation
    request.config.slowest_speed = 22.2
    # Distance of the leader to the slower vehicle in front in meters (front bumper to front bumper)
    request.config.slow_car_offset = 200
    # Set-up simulation
    request.config.sim = Simulation(scenario='testing', sumocfg=par.SUMO_TEST_CFG)
    request.config.platoon = Platoon(request.config.sim.plexe, request.config.sim.c2x)
    request.config.leader_pos = request.config.platoon.build(n=request.param, pos=130, speed=27.8)
    # Build id's of platoon members
    request.config.leader = par.LEADER
    request.config.follower = []
    for i in range(1, request.param):
        request.config.follower.append(f'{par.ID_PRE}_{i}')
    request.config.members = [request.config.leader] + request.config.follower
    # Expected and actual results
    request.config.expected = dict()
    request.config.results = dict()
    # Set parameters to thesis values
    par.MEAN_STEP_DURATION_MSG_DELIVERY = 0
    par.MAX_OVERTAKING_TIME = 45
    par.MIN_OVERTAKING_SPEED_DELTA = 2.7
    # Run test case
    yield
    # Print visited states
    request.config.platoon.print()
    # Assert results
    check_general(request)
    check_expected(request)


def check_general(request):
    """
    Checks simulation results for general requirements.
    """
    results = request.config.results
    members = request.config.members
    slowest_speed = request.config.slowest_speed
    # No collision occurred
    assert not results['collision']
    # Min gap is not smaller than the desired min gap of the platoon
    assert results['minGap'][0] >= par.INTER_VEHICLE_DISTANCE
    # Min speed >= slowest_speed and max speed <= desired speed
    speed_min = results[Key.SPEED_MIN]
    speed_max = results[Key.SPEED_MAX]
    for v_id in members:
        assert speed_max[v_id] <= par.SPEED * 1.05
        assert speed_min[v_id] >= slowest_speed * 0.95
    # Order of the platoon is still the same
    pos = results[Key.POSITION_STOP]
    # All members drove the same distance
    dist = results[Key.DISTANCE_DRIVEN]
    for i in range(0, len(members) - 1):
        assert pos[members[i]] > pos[members[i + 1]]
        assert dist[members[i]] == pytest.approx(dist[members[i + 1]], abs=0.2)


def check_expected(request):
    """
    Checks whether the expected results match the actual results.
    """
    expected = request.config.expected
    results = request.config.results
    assert len(expected) > 0
    for key in expected.keys():
        assert results[key] == expected[key]


def has_value(v_id: str, results: dict, key: Key, value, negate=False):
    """
    Asserts whether key for vehicle v_id has value. Does not test whether this is the only value.
    """
    # v_id is a list -> recursion
    if isinstance(v_id, list):
        for v in v_id:
            has_value(v, results, key, value, negate)
    # v_id is a single id
    else:
        # Key represents a list
        if isinstance(results[key][v_id], list):
            if negate:
                assert value not in results[key][v_id]
            else:
                assert value in results[key][v_id]
        # Key represents a scalar
        else:
            if negate:
                assert results[key][v_id] != value
            else:
                assert results[key][v_id] == value


def has_not_value(v_id: str, results: dict, key: Key, value):
    has_value(v_id, results, key, value, True)


def has_neighbor(v_id: str, results: dict, key: Key, value_before, value_after, direct=False, negate=False):
    """
    Asserts whether the index of value_before < index of value_after in the results for key.
    If neighbors=False then there may be other values between value_before and value_after.
    If neighbor=True then value_before and value_after have to be direct neighbors.
    """
    # v_id is a list -> recursion
    if isinstance(v_id, list):
        for v in v_id:
            has_neighbor(v, results, key, value_before, value_after, direct=direct, negate=negate)
    # v_id is a single id
    else:
        li = results[key][v_id]
        if not negate:
            # Both value_before and value_after have to be present
            assert value_before in li
            assert value_after in li
        else:
            # If value_before or value_after is not present in li, then they can not be neighbors
            if value_before not in li or value_after not in li:
                return
        # Direct Neighbors
        if direct:
            neighbor_found = False
            # Get all indices of value_before in li
            indices = [i for i, val in enumerate(li) if val == value_before]
            for index in indices:
                try:
                    if li[index + 1] == value_after:
                        neighbor_found = True
                except IndexError:
                    pass
            if not negate:
                assert neighbor_found
            else:
                assert not neighbor_found
        # Not necessarily direct neighbors
        else:
            index_before = li.index(value_before)
            index_after = len(li) - li[::-1].index(value_after) - 1
            if not negate:
                assert index_before < index_after
            else:
                assert index_before >= index_after


def has_not_neighbor(v_id: str, results: dict, key: Key, value_before, value_after, direct=False, negate=False):
    has_neighbor(v_id, results, key, value_before, value_after, direct=direct, negate=True)


def did_visit_state(request, s: State, leader=False, follower=False, negate=False):
    """
    Asserts whether state s was visited.
    """
    if leader:
        has_value(request.config.leader, request.config.results, Key.STATES_VISITED,
                  s, negate=negate)
    if follower:
        has_value(request.config.follower, request.config.results, Key.STATES_VISITED,
                  s, negate=negate)


def did_not_visit_state(request, s: State, leader=False, follower=False):
    did_visit_state(request, s, leader, follower, negate=True)


def did_overtake(request, v_id_overtaken):
    """
    Asserts whether the platoon did overtake vehicle v_id_overtaken. This means that all platoon members
    have a position > the position of the overtaken vehicle.
    """
    members = request.config.members
    results = request.config.results
    for v_id in members:
        assert results[Key.POSITION_STOP][v_id] > results[Key.POSITION_STOP][v_id_overtaken]


def did_not_overtake(request, v_id_overtaken):
    """
    Asserts whether the platoon did not overtake vehicle v_id_overtaken. That means that all platoon members
    have a position <= the position of the overtaken vehicle.
    """
    members = request.config.members
    results = request.config.results
    for v_id in members:
        assert results[Key.POSITION_STOP][v_id] <= results[Key.POSITION_STOP][v_id_overtaken]


def leaders_area_occupied(request, direction: Direction, negate=False):
    """
    Asserts whether the leader's maneuver area was occupied during the simulation.
    """
    if direction == Direction.LEFT:
        has_neighbor(request.config.leader, request.config.results, Key.STATES_VISITED,
                     State.ASSERT_OWN_AREAS, State.LANE_CHANGE_ABORTED, direct=True, negate=negate)
    else:
        has_neighbor(request.config.leader, request.config.results, Key.STATES_VISITED,
                     State.R_ASSERT_OWN_AREAS, State.R_LANE_CHANGE_ABORTED, direct=True, negate=negate)


def leaders_area_not_occupied(request, direction: Direction):
    leaders_area_occupied(request, direction, negate=True)


def leader_timeout_in_decision_phase(request, direction: Direction, negate=False):
    """
    Asserts whether the leader ran into a timeout during the simulation.
    """
    if direction == Direction.LEFT:
        has_neighbor(request.config.leader, request.config.results, Key.STATES_VISITED,
                     State.WAIT_FOR_RESPONSES, State.LANE_CHANGE_ABORTED, direct=True, negate=negate)
    else:
        has_neighbor(request.config.leader, request.config.results, Key.STATES_VISITED,
                     State.R_WAIT_FOR_RESPONSES, State.R_LANE_CHANGE_ABORTED, direct=True, negate=negate)


def no_leader_timeout_in_decision_phase(request, direction: Direction):
    leader_timeout_in_decision_phase(request, direction, negate=True)


def followers_area_occupied(request, direction: Direction, negate=False):
    """
    Asserts whether it was detected that the followers area is occupied:
    Leader changes from 'assert maneuver area' directly to 'lane change aborted'.
    Followers change from 'wait for decision' directly to 'idle'.
    """
    if direction == Direction.LEFT:
        has_neighbor(request.config.leader, request.config.results, Key.STATES_VISITED,
                     State.ASSERT_MANEUVER_AREA, State.LANE_CHANGE_ABORTED, direct=True, negate=negate)
        has_neighbor(request.config.follower, request.config.results, Key.STATES_VISITED,
                     State.WAIT_FOR_DECISION, State.IDLE, direct=True, negate=negate)
    else:
        has_neighbor(request.config.leader, request.config.results, Key.STATES_VISITED,
                     State.R_ASSERT_MANEUVER_AREA, State.R_LANE_CHANGE_ABORTED, direct=True, negate=negate)
        has_neighbor(request.config.follower, request.config.results, Key.STATES_VISITED,
                     State.R_WAIT_FOR_DECISION, State.R_IDLE, direct=True, negate=negate)


def followers_area_not_occupied(request, direction: Direction):
    followers_area_occupied(request, direction, negate=True)


def decision_not_always_received(request, direction: Direction, negate=False):
    """
    Asserts whether a decision was at least one time not received.
    """
    if direction == Direction.LEFT:
        has_neighbor(request.config.follower, request.config.results, Key.STATES_VISITED,
                     State.WAIT_FOR_DECISION, State.IDLE, direct=True, negate=negate)
    else:
        has_neighbor(request.config.follower, request.config.results, Key.STATES_VISITED,
                     State.R_WAIT_FOR_DECISION, State.R_IDLE, direct=True, negate=negate)


def decision_always_received(request, direction: Direction):
    decision_not_always_received(request, direction, negate=True)


def build_expected(request, exp: dict):
    """
    Builds the expected dictionary according to the platoon length that will be compared with the actual results.
    """
    leader = request.config.leader
    follower = request.config.follower
    expected = {}
    for key in exp:
        expected[key] = {}
        expected[key][leader] = exp[key]['leader']
        for v in follower:
            expected[key][v] = exp[key]['follower']
    return expected
