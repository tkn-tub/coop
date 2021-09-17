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

from enum import Enum, auto


class State(Enum):

    def __str__(self):
        return self.name.replace('_', ' ').lower()

    NO_STATE = auto()
    IDLE = auto()

    # Leaders overtaking states
    VEHICLE_AHEAD = auto()
    PASSING = auto()
    OVERTAKING_COMPLETE = auto()

    # Leaders lane change states
    ASSERT_OWN_AREAS = auto()  # also used by follower
    REQUEST_SENSOR_DATA = auto()
    WAIT_FOR_RESPONSES = auto()
    ASSERT_MANEUVER_AREA = auto()
    LANE_CHANGE_SAFE = auto()
    CHANGING_LANE = auto()  # also used by follower
    ABORT = auto()  # also used by follower
    CHANGING_BACK = auto()  # also used by follower
    LANE_CHANGE_ABORTED = auto()
    LANE_CHANGE_COMPLETE = auto()  # also used by follower

    # Followers lane change states
    WAIT_FOR_DECISION = auto()
    IN_ORIGINAL_LANE = auto()
    IN_OVERTAKING_LANE = auto()
    LANE_CHANGED = auto()

    # Pseudo states to be able to distinguish the states of a left and a right lane change.
    # Not used for the algorithm but for testing.

    R_IDLE = auto()

    # Leaders overtaking states
    R_VEHICLE_AHEAD = auto()
    # LANE_CHANGE = auto()
    R_PASSING = auto()
    R_OVERTAKING_COMPLETE = auto()

    # Leaders lane change states
    R_ASSERT_OWN_AREAS = auto()  # also used by follower
    R_REQUEST_SENSOR_DATA = auto()
    R_WAIT_FOR_RESPONSES = auto()
    R_ASSERT_MANEUVER_AREA = auto()
    R_LANE_CHANGE_SAFE = auto()
    R_CHANGING_LANE = auto()  # also used by follower
    R_ABORT = auto()  # also used by follower
    R_CHANGING_BACK = auto()  # also used by follower
    R_LANE_CHANGE_ABORTED = auto()
    R_LANE_CHANGE_COMPLETE = auto()  # also used by follower

    # Followers lane change states
    R_WAIT_FOR_DECISION = auto()
    R_IN_ORIGINAL_LANE = auto()
    R_IN_OVERTAKING_LANE = auto()
    R_LANE_CHANGED = auto()
