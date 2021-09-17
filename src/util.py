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

import math

import traci
from plexe import POS_X, POS_Y

import parameter as par
from direction import Direction


def add_vehicle(plexe, v_id, route_id, pos, lane, speed, vtype, color=None):
    """
    Adds a vehicle with v_id to the simulation.

    :param plexe: Plexe
    :param v_id: ID of the vehicle
    :param route_id: ID of the route the vehicle has to use
    :param pos: departing position of the vehicle
    :param lane: lane of the vehicle
    :param speed: speed of the vehicle
    :param vtype: vType of the vehicle
    :param color: color of the vehicle
    """
    if plexe.version[0] >= 1:
        # All your base are belong to us
        if traci.route.getEdges(route_id)[0] == 'intermediate':
            zero_wing = traci.lane.getLastStepVehicleIDs('intermediate_0')
            for v in zero_wing:
                traci.vehicle.remove(v)
        # Add vehicle
        traci.vehicle.add(vehID=v_id, routeID=route_id, departPos=str(pos), departSpeed=str(speed),
                          departLane=str(lane), typeID=vtype)
        if color is not None:
            traci.vehicle.setColor(v_id, color)
    else:
        traci.vehicle.add(v_id, route_id, pos=pos, speed=speed, lane=lane, typeID=vtype)


def remove_vehicle(v_id):
    traci.vehicle.remove(v_id)


def move_vehicle(v_id, lane_id, pos):
    traci.vehicle.moveTo(v_id, lane_id, pos)


def get_distance(plexe, v1, v2):
    v1_data = plexe.get_vehicle_data(v1)
    v2_data = plexe.get_vehicle_data(v2)
    return math.sqrt((v1_data[POS_X] - v2_data[POS_X]) ** 2 +  # noqa 504
                     (v1_data[POS_Y] - v2_data[POS_Y]) ** 2) - 4


def set_poi(label, x):
    result = dict()
    result['label'] = label
    result['x'] = x
    return result


def get_leader(v_id):
    vehicle = traci.vehicle.getLeader(v_id, par.RADAR_FRONT)
    if vehicle is not None:
        # Vehicle is detectable by radar and is not a platoon member
        if vehicle[1] <= par.RADAR_FRONT and not vehicle[0].startswith(par.ID_PRE):
            return vehicle
    return None


def get_follower(v_id):
    vehicle = traci.vehicle.getFollower(v_id, par.RADAR_REAR)
    if vehicle[0] != '':
        # Vehicle is detectable by radar and is not a platoon member
        if vehicle[1] <= par.RADAR_REAR and not vehicle[0].startswith(par.ID_PRE):
            return vehicle
    return None


def get_neighbor(v_id, direction: Direction, front: bool, select='closest', exclude_platoon=True):
    """
    Returns (v_id, distance) of the vehicle in the left or right lane of the given v_id that is detectable by the
    vehicles radar (front defines longitudinal direction: True=leader, False=follower).
    """
    result = None
    if front:
        radar_length = par.RADAR_FRONT
        if direction == Direction.LEFT:
            vehicle = traci.vehicle.getLeftLeaders(v_id)
        elif direction == Direction.RIGHT:
            vehicle = traci.vehicle.getRightLeaders(v_id)
        else:
            vehicle = traci.vehicle.getLeader(v_id)
            if vehicle is None:
                vehicle = tuple()
            else:
                vehicle = tuple([vehicle])
    else:
        radar_length = par.RADAR_REAR
        if direction == Direction.LEFT:
            vehicle = traci.vehicle.getLeftFollowers(v_id)
        elif direction == Direction.RIGHT:
            vehicle = traci.vehicle.getRightFollowers(v_id)
        else:
            # Not needed -> use get_follower()
            assert False
    # Remove platoon vehicles
    if exclude_platoon:
        vehicle = [v for v in vehicle if not v[0].startswith(par.ID_PRE)]
    # Remove vehicles that are out of radar range
    vehicle = [v for v in vehicle if v[1] <= radar_length]
    # At least one detectable vehicle
    if len(vehicle) > 0:
        # select vehicle to return
        if select == 'all':
            result = vehicle
        elif select == 'closest':
            min_dist = vehicle[0][1]
            min_index = 0
            for i, v in enumerate(vehicle):
                if vehicle[i][1] < min_dist:
                    min_dist = vehicle[i][1]
                    min_index = i
            result = vehicle[min_index]
        elif select == 'slowest':
            min_speed = get_speed(vehicle[0][0])
            min_index = 0
            for i, v in enumerate(vehicle):
                if get_speed(vehicle[i][0]) < min_speed:
                    min_speed = get_speed(vehicle[i][0])
                    min_index = i
            result = vehicle[min_index]
        else:
            # Undefined selector
            assert False
    return result


def change_sublane(v_id, lat_dist: float):
    traci.vehicle.changeSublane(v_id, lat_dist)


def get_time():
    return traci.simulation.getTime()


def get_speed(v_id):
    return traci.vehicle.getSpeed(v_id)


def set_speed(v_id, speed: float):
    traci.vehicle.setSpeed(v_id, speed)


def get_max_speed(v_id):
    return traci.vehicle.getMaxSpeed(v_id)


def get_lane_max_speed(v_id):
    lane = get_lane_id(v_id)
    return traci.lane.getMaxSpeed(lane)


def set_color(v_id, color: (int, int, int)):
    traci.vehicle.setColor(v_id, color)


def get_lane_id(v_id):
    return traci.vehicle.getLaneID(v_id)


def get_lane_index(v_id):
    return traci.vehicle.getLaneIndex(v_id)


def get_lane_width(v_id):
    lane_id = traci.vehicle.getLaneID(v_id)
    return traci.lane.getWidth(lane_id)


def get_lateral_speed(v_id):
    return traci.vehicle.getLateralSpeed(v_id)


def get_lateral_max_speed(v_id):
    return traci.vehicle.getMaxSpeedLat(v_id)


def get_lateral_offset(v_id):
    return traci.vehicle.getLateralLanePosition(v_id)


def get_distance_to_original_lane(v_id):
    return traci.vehicle.getPosition(v_id)[1] - par.LAT_POS_START


def set_lane_change_mode(v_id, lc_mode):
    traci.vehicle.setLaneChangeMode(v_id, lc_mode)


def is_centered(v_id) -> bool:
    lat_speed = get_lateral_speed(v_id)
    offset = traci.vehicle.getLateralLanePosition(v_id)
    if abs(offset) <= 0.01 and abs(lat_speed) == 0:
        return True
    else:
        return False


def get_tau(v_id):
    return traci.vehicle.getTau(v_id)


def get_max_accel(v_id):
    return traci.vehicle.getAccel(v_id)


def get_length(v_id):
    return traci.vehicle.getLength(v_id)


def get_vtype_length(vtype):
    return traci.vehicletype.getLength(vtype)


def set_speed_mode(v_id, mode):
    traci.vehicle.setSpeedMode(v_id, mode)


def track_vehicle(view, v_id):
    traci.gui.trackVehicle(view, v_id)


def get_x_pos(v_id):
    return traci.vehicle.getPosition(v_id)[0]


def became_slower(old_speed, new_speed):
    if old_speed - new_speed > par.ROUND_PRECISION:
        return True
    else:
        return False


def became_faster(old_speed, new_speed):
    if new_speed - old_speed > par.ROUND_PRECISION:
        return True
    else:
        return False
