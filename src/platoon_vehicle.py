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
from c2x import C2X
from direction import Direction
from key import Key
from message import Message
from state import State


class PlatoonVehicle:
    """
    Class that handles platoon members and followers.
    """

    def __init__(self, v_id: str, c2x: C2X):
        self.v_id = v_id
        self.step = 0
        self.timer = 0
        self.state = State.NO_STATE
        self.direction = Direction.LEFT
        self.in_lane_change = False
        self.begin_lane_change = False
        self.in_change_back = False
        self.begin_change_back = False
        self.msg_queue = []
        self.rec_data = {Key.STATES_VISITED: [],
                         Key.OT_TIME: [],
                         Key.LANES_VISITED: [],
                         Key.SPEED_MIN: float('inf'),
                         Key.SPEED_MAX: float('-inf'),
                         Key.MIN_GAP: float('inf')}
        # Messaging system
        self.c2x = c2x
        # Reset timer and overtaking state
        self.set_timer(0)
        self.set_state(State.IDLE)
        # Lane change from lane lc_from to lane lc_to
        self.lc_from = 0
        self.lc_to = 0
        # Platoon did cross the lane markings -> self.in_new_lane = True
        self.in_new_lane = False
        self.lane_offset_last_step = 0
        self.abort_message = ''

    def detect_lane_crossing(self):
        """
        Detects whether the platoon crosses the lane markings during a lane change maneuver.
        """
        # Detect crossing of lane markings
        curr_offset = util.get_lateral_offset(self.v_id)
        # Lane crossed
        if abs(curr_offset) > par.LANE_OFFSET_MIN and self.lane_offset_last_step * curr_offset < 0:
            self.in_new_lane = True
        self.lane_offset_last_step = curr_offset

    def safety_dist_platoon(self):
        """
        Computes the necessary safety distance (in meters) of the platoon to a (non-platooning) vehicle in front.

        :return: Necessary safety distance in meters
        """
        d_safety = util.get_tau(self.v_id) * util.get_speed(self.v_id)
        # Raise d_safety a little bit when not in a lane change to avoid oscillation
        if not self.in_lane_change:
            d_safety *= (1 + par.OSCILLATION_MOD)
        return d_safety

    def alpha_area_free(self, debug=True) -> bool:
        """
        Returns True if the FL (or FR, depending on self.direction) area is considered free, False otherwise. The
        area is considered free if no vehicle is detected in that area or the closest vehicle in that area is at
        least the platoon's safety distance away.

        :return: True if FL (or FR, depending on self.direction) area is considered free, False otherwise
        """
        is_free = True
        # Get relevant vehicle in FL (or FR) area, platoon is changing lanes and already in new lane
        if self.in_lane_change and self.in_new_lane:
            neighbor = util.get_leader(self.v_id)
        else:
            neighbor = util.get_neighbor(self.v_id, self.direction, front=True)
        if neighbor is not None:
            if neighbor[1] < self.safety_dist_platoon():
                is_free = False
                self.debug_print(f'F{self.direction.short()} area is occupied by {neighbor[0]} with distance '
                                 f'{round(neighbor[1], 2)} < safety={round(self.safety_dist_platoon(), 2)}')
            else:
                if debug:
                    self.debug_print(f'F{self.direction.short()} area is free ({neighbor[0]} with distance '
                                     f'{round(neighbor[1], 2)} >= '
                                     f'safety={round(self.safety_dist_platoon(), 2)})')
        else:
            if debug:
                self.debug_print(f'F{self.direction.short()} area is free (no vehicle detected)')
        return is_free

    def d_min(self, a: float, follower) -> float:
        """
        Returns the minimum distance the vehicle in the RL (or RR) area has to be away from the platoon to ensure
        that this vehicle only has to break with a (negative) acceleration of at most <a> and that the safety distance
        is observed.

        :param a: Minimum allowed (negative) acceleration of the RL (or RR) vehicle
        :param follower: Tuple of follower's id and distance
        :return: Minimum distance to the RL (or RR) vehicle so the beta area is considered free
        """
        v_p = util.get_speed(self.v_id)
        v_r = util.get_speed(follower[0])
        T_r = par.R_REACT
        T_g = par.R_TIME_GAP
        if v_r > v_p and a < 0:
            result = (-1 / (2 * a)) * (v_p - v_r) ** 2 + v_r * T_r + v_p * T_g
        elif v_r <= v_p and a <= 0:
            result = v_r * (T_r + T_g)
        else:
            result = float('inf')
        # Take truck min gap of 50m into account when changing to the right
        if self.direction == Direction.RIGHT:
            result = max(result, par.TRUCK_MIN_GAP)
        # Raise d_min a little bit when not in a lane change to avoid oscillation
        if not self.in_lane_change:
            result *= (1 + par.OSCILLATION_MOD)
        return result

    def beta_area_free(self, debug=True) -> bool:
        """
        Returns True if the RL (or RR, depending on self.direction) area is considered free, False otherwise.
        The area is considered free if no vehicle is detected in that area or the closest vehicle is at least d_min
        away.

        :return: True if RL (or RR, depending on self.direction) area is considered free, False otherwise
        """
        is_free = True
        # Max accepted (negative) acceleration by neighbor follower
        if self.in_lane_change:
            threshold = par.MAX_ACCEL_R_VEHICLE_ABORT
        else:
            if self.direction == Direction.LEFT:
                threshold = par.MAX_ACCEL_VEHICLE_LEFT
            else:
                threshold = par.MAX_ACCEL_VEHICLE_RIGHT
        # Get relevant vehicle in RL (or RR) area
        if self.in_lane_change and self.in_new_lane:
            # Platoon in lane change and already in new lane
            neighbor = util.get_follower(self.v_id)
        else:
            # Platoon not in lange change or in lane change and still in old lane
            neighbor = util.get_neighbor(self.v_id, self.direction, front=False)
        if neighbor is not None:
            # Compute necessary distance to closest vehicle in RL (or RR) area to ensure safety distance
            min_dist = self.d_min(threshold, neighbor)
            if debug:
                self.debug_print(f"R{self.direction.short()} '{neighbor[0]}' needs distance "
                                 f'{round(min_dist, 2)} (has {round(neighbor[1], 2)}) '
                                 f'to ensure safety distance (threshold={threshold})')
            if neighbor[1] < min_dist:
                self.debug_print(f'R{self.direction.short()} area is occupied by {neighbor[0]} with distance '
                                 f'{round(neighbor[1], 2)} (needs {round(min_dist, 2)})')
                is_free = False
            else:
                if debug:
                    self.debug_print(f'R{self.direction.short()} area is free ({neighbor[0]})')
        else:
            if debug:
                self.debug_print(f'R{self.direction.short()} area is free (no vehicle detected)')
        return is_free

    def exec_lane_change(self):
        """
        Orders the vehicle to changes lanes in the direction of self.direction.
        """
        # Vehicle is already in target lane
        if self.in_new_lane:
            lat_dist = -1 * util.get_lateral_offset(self.v_id)
        # Vehicle is still in original lane
        else:
            if self.direction == Direction.LEFT:
                lat_dist = par.LANE_WIDTH - util.get_lateral_offset(self.v_id)
            else:
                lat_dist = -1 * (par.LANE_WIDTH + util.get_lateral_offset(self.v_id))
        util.change_sublane(self.v_id, lat_dist)
        self.debug_print(f'Lane change from lane {self.lc_from} to {self.lc_to} started (lat={round(lat_dist, 2)})')

    def set_state(self, state: State):
        """
        Sets the lane change state of the vehicle according to the cooperative overtaking algorithm.
        """
        self.state = state
        # Distinguish between left and right lane change maneuver states
        # (only for testing the algorithm, not needed for the actual algorithm)
        if self.direction == Direction.LEFT or state == State.PASSING or state == State.OVERTAKING_COMPLETE:
            self.rec_data[Key.STATES_VISITED].append(state)
        else:
            self.rec_data[Key.STATES_VISITED].append(State['R_' + state.name])
        self.debug_print(f'*** Switching to state {state.value}: {state} ***')

    def set_timer(self, value=par.MAX_TIMER):
        """
        Sets the timer to its standard value. If no value is given, par.MAX_TIMER is used.
        """
        self.timer = value
        self.debug_print(f'Timer set to {self.timer}')

    def decrement_timer(self):
        """
        Reduces the timer of each vehicle by 1 until it reaches 0.
        """
        if self.timer > 0:
            self.timer -= 1

    def record(self):
        """
        Records data of the platoon vehicle.
        """
        # Current lane
        lane = util.get_lane_index(self.v_id)
        if len(self.rec_data[Key.LANES_VISITED]) == 0 or self.rec_data[Key.LANES_VISITED][-1] != lane:
            self.rec_data[Key.LANES_VISITED].append(lane)
        # Min and max speed
        speed = round(util.get_speed(self.v_id), 2)
        if self.rec_data[Key.SPEED_MIN] > speed:
            self.rec_data[Key.SPEED_MIN] = speed
        if self.rec_data[Key.SPEED_MAX] < speed:
            self.rec_data[Key.SPEED_MAX] = speed

    def front_or_rear_member_not_following(self):
        """
        Returns True if the platoon member in front or in rear is not following laterally anymore
        (meaning the lateral offset is bigger than par.MEMBER_MAX_OFFSET).

        :return: True if front or rear member is not following laterally
        """
        # Own lateral position diverges too much from the leader's lateral position
        own_index = int(self.v_id.split('_')[1])
        own_lat_pos = abs(util.get_lateral_offset(self.v_id))
        if own_index > 0:
            front_id = par.ID_PRE + '_' + str(own_index - 1)
            front_lat_pos = abs(util.get_lateral_offset(front_id))
            if abs(own_lat_pos - front_lat_pos) > par.MEMBER_MAX_OFFSET:
                return True
        if own_index < par.PLATOON_MEMBERS - 1:
            rear_id = par.ID_PRE + '_' + str(own_index + 1)
            rear_lat_pos = abs(util.get_lateral_offset(rear_id))
            if abs(own_lat_pos - rear_lat_pos) > par.MEMBER_MAX_OFFSET:
                return True
        return False

    def next_step(self):
        """
        Main next steps that are executed for both the leader and the followers.
        """
        self.step += 1
        # Detect crossing of a lane marking
        if self.in_lane_change or self.in_change_back:
            self.detect_lane_crossing()
        self.record()
        # Discard earlier received messages
        self.msg_queue.clear()
        self.receive_messages()
        self.decrement_timer()

    # *** Messaging ***

    def receive_messages(self):
        """
        Receives all relevant messages and adds them to the queue.
         """
        self.msg_queue.extend(self.c2x.receive(self.v_id, self.step))
        for msg in self.msg_queue:
            self.debug_print(f'Message in queue from {msg["src"]}: {msg["msg"].name}')

    def get_messages_by_type(self, msg_type: Message):
        """
        Returns all messages from the queue that are of type msg. Deletes these messages from the queue.
        """
        result = [msg for msg in self.msg_queue if msg['msg'] == msg_type]
        self.msg_queue = [msg for msg in self.msg_queue if msg['msg'] != msg_type]
        return result

    def send_message(self, dest, msg: Message, data=None):
        """
        Sends a message to dest via the C2X communication system.
        """
        self.c2x.send(self.v_id, dest, msg, data, self.step)
        self.debug_print(f'Message sent to {dest}: \'{msg.name}\'')

    # *** Debug methods ***

    def print(self, substates=False):
        """
        Print some information about the vehicle (for debugging purposes).
        """
        print(f'  {self.v_id}: state sequence: {self.rec_data[Key.STATES_VISITED]}')

    def debug_print(self, message):
        """
        Prints debug information to stdout.
        """
        if par.DEBUG:
            print(f'({self.step}) {self.v_id}: {message}')

    # *** States that apply to both the leader and the follower ***

    def changing_lane(self):
        """
        State "changing lane"
        """
        # Begin lane change
        if self.begin_lane_change:
            self.begin_lane_change = False
            self.in_lane_change = True
            self.in_new_lane = False
            self.lc_from = util.get_lane_index(self.v_id)
            self.lc_to = self.lc_from + self.direction.value
            self.exec_lane_change()
        # During lane change
        elif self.in_lane_change:
            # Handle disruptions during lane change
            if util.get_lateral_speed(self.v_id) != 0:
                # FL (or FR) area occupied
                if not self.alpha_area_free(debug=False):
                    self.begin_change_back = True
                    self.abort_message = 'Front area occupied'
                # RL (or RR) area occupied
                if not self.beta_area_free(debug=False):
                    self.begin_change_back = True
                    self.abort_message = 'Rear area occupied'
                # Unreliable messaging: another platoon member does not change lanes
                if self.front_or_rear_member_not_following():
                    self.begin_change_back = True
                    self.abort_message = 'A member is not following laterally anymore'
            # Lane change done
            elif self.in_new_lane and util.is_centered(self.v_id):
                self.lc_from = self.lc_to
                self.in_lane_change = False

    def lane_change_complete(self):
        """
        State: "lane change complete"
        """
        self.in_new_lane = False
        self.direction = self.direction.flip()

    def changing_back(self):
        """
        State: "changing back"
        """
        # Begin changing back
        if self.begin_change_back:
            self.begin_change_back = False
            self.in_change_back = True
            self.in_lane_change = False
            self.direction = self.direction.flip()
            self.lc_from = util.get_lane_index(self.v_id)
            # Platoon already in new lane
            if self.in_new_lane:
                self.lc_to = self.lc_from + self.direction.value
            else:
                self.lc_to = self.lc_from
            # When already in new lane -> now in old lane
            # When still in old lane -> now in new lane
            self.in_new_lane = not self.in_new_lane
            self.exec_lane_change()
        # During changing back
        elif self.in_change_back:
            # Lane change done
            if self.in_new_lane and util.is_centered(self.v_id):
                self.direction = self.direction.flip()
                self.in_change_back = False
