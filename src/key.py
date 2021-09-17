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

class Key:

    # Platoon data
    LANES_VISITED = '1'
    STATES_VISITED = '2'
    SUBSTATES_VISITED = '3'
    SPEED_MIN = '4'
    SPEED_MAX = '5'
    MIN_GAP = '6'
    DISTANCE_DRIVEN = '9'
    OT_TIME = '10'

    # Simulation data
    POSITION_START = '7'
    POSITION_STOP = '8'
