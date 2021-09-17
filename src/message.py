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


class Message(Enum):

    REQ_SENSOR_DATA = auto()
    RESP_SENSOR_DATA = auto()
    OVERTAKING_IS_SAFE = auto()

    BEGIN_LANE_CHANGE = auto()
    LANE_CHANGE_COMPLETE = auto()
    ABORT = auto()
    ABORT_COMPLETE = auto()
