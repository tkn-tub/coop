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

import os
import sys

# Standard simulation configuration
SUMO_TEST_CFG = "cfg/freeway_test.sumocfg"

# Path to SUMO home directory
if 'SUMO_HOME' not in os.environ:
    sys.exit("Please declare environment variable 'SUMO_HOME'")
SUMO_HOME = os.environ['SUMO_HOME']
tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
sys.path.append(tools)

SUMO_PARAMS = ["--lateral-resolution", "0.8",
               "--quit-on-end"]

# Set to False for faster simulations
SHOW_GUI = False
SHOW_SIMUWATCH = False

# Print debug information
DEBUG = False

# Distance the platoon will drive before the simulation ends (in meters)
DISTANCE_TO_DRIVE = 3200

# Simulation step length (in seconds)
STEP_LENGTH = 0.01

# Recordings per second
REC_PER_SEC = 10

# Seed for random numbers (SUMO default value is 23423)
RND_SEED = '23422'

# Lane width (info value, not a parameter)
LANE_WIDTH = 3.2

# Lateral start position of platoon (info value, not a parameter)
LAT_POS_START = 42

# Minimum lateral offset (in meters) when to look for the crossing of lane markings
LANE_OFFSET_MIN = 0.5

# Oscillation modifier to avoid oscillation
OSCILLATION_MOD = 0.01

# ----- Messaging parameters ----- #

# Reliability of the messaging system (exponentially distributed)
# 5 for conference setup, 0 for thesis and tests
MEAN_STEP_DURATION_MSG_DELIVERY = 5

# ----- Algorithm parameters ----- #

# The distance from the most left edge of the road to the point where the platoon has to be on the right lane
# (e.g., to exit the freeway)
# NEXT_NAV_POINT = 700

# Turn the algorithm on or off (do not enable in combination with Plexe algorithm)
OVERTAKING_ALGORITHM_ON = True

# Turn the Plexe overtaking algorithm on or off (do not enable in combination with algorithm)
PLEXE_OVERTAKING_ON = False

# Exit after one full overtaking maneuver
EXIT_ON_OVERTAKING = False

# Plexe platoon controller
# ACC = 1, CACC = 2, PLOEG = 4, CONSENSUS = 5
PLEXE_LEADER_CONTROLLER = 1
PLEXE_FOLLOWER_CONTROLLER = 2

# Maximum number of simulation steps for an overtaken vehicle not being detected during a lane change to not cause
# abortion of the lane change (if an overtaken vehicle is not detected for MAX_NOT_DETECTED + 1 simulation steps,
# the lane change will be aborted)
# Necessary, because SUMO does not detect the overtaken vehicle when crossing two lanes
MAX_NOT_DETECTED = 3

# Maximum power of back-off. Back-off is computed as 2 ** (back-off), back-off starts at MIN_BACKOFF
# 2 ** MIN_BACKOFF should be bigger than MAX_TIMER
MAX_BACKOFF = 8
MIN_BACKOFF = 5

# Timeout in overtaking algorithm steps
MAX_TIMER = 20

# Speed deviations smaller than ROUND_PRECISION will be handled as equal speeds
ROUND_PRECISION = 0.05

# Maximum length of one truck (in meters)
TRUCK_MAX_LENGTH = 18.75

# Threshold (in seconds) for an overtaking maneuver
# 100000000 for conference setup, 45 for thesis and tests
MAX_OVERTAKING_TIME = 100000000

# Threshold speed delta of platoon and slower vehicle needed to allow overtaking
# 0.1 for conference setup, 2.7 for thesis and tests
MIN_OVERTAKING_SPEED_DELTA = 0.1

# Overtake if distance to slower vehicle is x * safety distance
FACTOR_SAFETY_DISTANCE = 100

# Maximum accepted acceleration (in m/s^2) of closest RL vehicle to avoid collision with platoon when platoon is
# deciding to merge in his lane (lane change left)
MAX_ACCEL_VEHICLE_LEFT = -1.0
# Maximum accepted acceleration (in m/s^2) of closest Rx vehicle to avoid collision with platoon when platoon is
# already merging in his lane
MAX_ACCEL_R_VEHICLE_ABORT = -3.5
# Maximum accepted acceleration (in m/s^2) of closest RR vehicle to avoid collision with platoon when platoon is
# deciding to merge in his lane (lane change right)
MAX_ACCEL_VEHICLE_RIGHT = 0

# Reaction time of other vehicle's drivers (in seconds)
R_REACT = 1

# Desired time gap of other vehicle's drivers (in seconds)
R_TIME_GAP = 0.8

# Min safety distance (in meter) for a non-platooning truck on a German freeway
TRUCK_MIN_GAP = 50

# Time headway (in seconds) of a non-platooning truck (reaction time + desired time gap)
TRUCK_HEADWAY = 1.8

# Time that the platoon should drive on the original lane after returning from the overtaking lane before
# overtaking again
TIME_STAY_IN_ORIGINAL_LANE = 10

# The lateral offset threshold for a platoon member considered not following anymore (in meters)
MEMBER_MAX_OFFSET = 0.4

# ----- Platoon parameters ----- #

# Id of platoon leader
LEADER = 'Platoon_0'

# Prefix for platoon members
ID_PRE = 'Platoon'

# Platoon route
PLATOON_ROUTE = 'freeway'

# Platoon length (is overwritten by Platoon.build())
PLATOON_MEMBERS = 4

# Inter-vehicle distance
INTER_VEHICLE_DISTANCE = 5

# Damping ratio
XI = 1.0

# Bandwidth of the controller
OMEGA_N = 0.2

# Leader/front vehicle weighting factor
C1 = 0.5

# Cruise control speed (in m/s)
SPEED = 30.55

# ACC headway time in s
ACC_HEADWAY = 1.0

# Radar length (in m)
RADAR_FRONT = 160
RADAR_REAR = 80
