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

from math import sqrt

import parameter as par
import util
from c2x import C2X
from direction import Direction
from key import Key
from message import Message
from platoon_vehicle import PlatoonVehicle
from state import State


def safety_dist_slower(v_slower):
    """
    Computes the necessary safety distance (in meters) of a non-platooning truck.
    Formula: maximum of required min distance of a truck on German freeways and his anticipated desired time headway

    @return: min safety distance in meters
    """
    return max(par.TRUCK_MIN_GAP, par.TRUCK_HEADWAY * v_slower)


class PlatoonLeader(PlatoonVehicle):
    """
    Class that handles the platoon leader.
    """

    def __init__(self, v_id: str, wifi: C2X):
        super().__init__(v_id, wifi)
        self.backoff = par.MIN_BACKOFF
        self.follower = []
        # Desired and maximum speed of the platoon
        self.v_desired = 0.0
        self.max_speed = 0.0
        # Max acceleration of the platoon
        self.max_accel = 0.0
        self.waiting_for_reply = []
        self.safety_data = []
        # Predicted time for an overtaking maneuver
        self.ot_predicted = 0  # in seconds
        # Simulation time when overtaking maneuver was started (transition from 'vehicle ahead' to 'assert own areas')
        self.ot_started = -1
        self.detection_threshold = 0

    def platoon_length(self):
        """
        Returns the length of the platoon (front bumper of the leader to rear bumper of the last follower).

        @return: length of the platoon in meters
        """
        return util.get_x_pos(self.v_id) - util.get_x_pos(self.follower[-1]) + util.get_length(self.follower[-1])

    def set_backoff_timer(self):
        """
        Sets the exponential back-off timer.
        """
        self.set_timer(2 ** self.backoff)
        if self.backoff < par.MAX_BACKOFF:
            self.backoff += 1

    def t_over(self, v_max_ot_lane, v_slower, dist_slower) -> float:
        """
        Returns the time (in seconds) needed to complete an overtaking maneuver of a truck with maximum length.

        :param v_max_ot_lane: max speed the platoon can/will drive on the overtaking lane
        :param v_slower: speed of the vehicle to be overtaken
        :param dist_slower: distance to the vehicle to be overtaken
        """
        a_max = util.get_max_accel(self.v_id)
        v_p = util.get_speed(self.v_id)
        # Reduce dist_slower when changing to original lane
        if self.direction == Direction.RIGHT:
            t_lane_change = (
                util.get_distance_to_original_lane(self.v_id) / util.get_lateral_max_speed(self.v_id)
                + par.TIME_STAY_IN_ORIGINAL_LANE  # noqa 503
            )
            d_lane_change = (util.get_speed(self.v_id) - v_slower) * t_lane_change
            dist_slower = dist_slower - d_lane_change
        # Platoon is already at most the safety distance away from to the slower vehicle -> t_over = 0
        # This can happen while driving in the overtaking lane and taking par.TIME_STAY_IN_ORIGINAL_LANE
        # into account
        if dist_slower < self.safety_dist_platoon():
            return 0
        l_total = dist_slower + par.TRUCK_MAX_LENGTH + safety_dist_slower(v_slower) + self.platoon_length()
        v_over = v_slower + sqrt((v_p - v_slower) ** 2 + 2 * a_max * l_total)
        try:
            if v_over <= v_max_ot_lane:
                t_over = (1 / a_max) * (v_slower - v_p + sqrt((v_p - v_slower) ** 2 + 2 * a_max * l_total))
            else:
                t_over = (
                    (l_total / (v_max_ot_lane - v_slower)) *  # noqa 504
                    (1 + (v_max_ot_lane - v_p) ** 2 / (2 * a_max * l_total))
                )
        except ZeroDivisionError:
            return float('inf')
        # Add time needed for lane change
        return t_over + par.LANE_WIDTH / util.get_lateral_max_speed(self.v_id)

    def lane_speed(self, d: Direction) -> float:
        """
        Returns the maximum speed on the <direction> lane. The maximum speed is defined as the minimum of the
        platoon's desired speed and the speed limit of that lane.

        :return: Maximum speed of the lane
        """
        vehicle = util.get_neighbor(self.v_id, d, front=True, select='slowest')
        if vehicle is not None:
            # return min(self.v_desired, util.get_speed(vehicle[0]), util.get_lane_max_speed(self.v_id))
            # Parameter study: ignore FL vehicle
            return min(self.v_desired, util.get_lane_max_speed(self.v_id))
        else:
            return min(self.v_desired, util.get_lane_max_speed(self.v_id))

    def next_step(self):
        """
        Executes one step of the cooperative overtaking algorithm.
        """
        super().next_step()

        # --- Overtaking states ---

        # State: idle
        if self.state == State.IDLE:
            self.idle()

        # State: vehicle ahead
        elif self.state == State.VEHICLE_AHEAD:
            self.vehicle_ahead()

        # State: passing
        elif self.state == State.PASSING:
            self.passing()

        # State: overtaking complete
        elif self.state == State.OVERTAKING_COMPLETE:
            self.overtaking_complete()

        # --- Lane change states ---

        # State: assert alpha and beta areas
        elif self.state == State.ASSERT_OWN_AREAS:
            self.assert_own_areas()

        # State: request sensor data
        elif self.state == State.REQUEST_SENSOR_DATA:
            self.request_sensor_data()

        # State: wait for responses
        elif self.state == State.WAIT_FOR_RESPONSES:
            self.wait_for_responses()

        # State: assert maneuver area
        elif self.state == State.ASSERT_MANEUVER_AREA:
            self.assert_maneuver_area()

        # State: lane change aborted
        elif self.state == State.LANE_CHANGE_ABORTED:
            self.lane_change_aborted()

        # State: lane change safe
        elif self.state == State.LANE_CHANGE_SAFE:
            self.lane_change_safe()

        # State: changing lane
        elif self.state == State.CHANGING_LANE:
            self.changing_lane()

        # State: lane change complete
        elif self.state == State.LANE_CHANGE_COMPLETE:
            self.lane_change_complete()

        # State: abort
        elif self.state == State.ABORT:
            self.abort()

        # State: changing back
        elif self.state == State.CHANGING_BACK:
            self.changing_back()

    def idle(self):
        """
        State "idle"
        """
        self.safety_data.clear()
        self.msg_queue.clear()
        vehicle_ahead = util.get_leader(self.v_id)
        if vehicle_ahead is not None:
            self.direction = Direction.LEFT
            self.timer = 0
            self.debug_print(f"vehicle '{vehicle_ahead[0]}' detected")
            self.set_state(State.VEHICLE_AHEAD)

    def should_overtake(self, vehicles) -> (bool, float, float, float):
        """
        Returns True if it is useful and possible to overtake ot_vehicle, False otherwise.
        """
        if vehicles is None or len(vehicles) == 0:
            v_max_ot_lane = -1
            v_ot_vehicle_all = -1
            ot_time_all = -1
        else:
            v_ot_vehicle_all = []
            ot_time_all = []
            if self.in_lane_change or self.in_change_back:
                # Platoon is changing to the left
                if self.direction == Direction.LEFT:
                    # Platoon is already mostly in new lane
                    if self.in_new_lane:
                        direct_ot_lane = Direction.SAME
                    else:
                        direct_ot_lane = Direction.LEFT
                # Platoon is changing to the right
                else:
                    # Platoon is already mostly in new lane
                    if self.in_new_lane:
                        direct_ot_lane = Direction.LEFT
                    else:
                        direct_ot_lane = Direction.SAME
            else:
                if self.direction == Direction.LEFT:
                    direct_ot_lane = Direction.LEFT
                else:
                    direct_ot_lane = Direction.SAME
            v_max_ot_lane = min(self.lane_speed(direct_ot_lane), self.v_desired)
            # Check for each given vehicle whether to overtake it
            for ot_vehicle in vehicles:
                v_ot_vehicle = util.get_speed(ot_vehicle[0])
                dist_ot_vehicle = ot_vehicle[1]
                ot_time = self.t_over(v_max_ot_lane, v_ot_vehicle, dist_ot_vehicle)
                speed_threshold = par.MIN_OVERTAKING_SPEED_DELTA
                time_threshold = par.MAX_OVERTAKING_TIME
                # Raise thresholds when in original lane or when changing to the original lane to avoid oscillation
                if (self.direction == Direction.LEFT and not self.in_lane_change) or \
                        (self.direction == Direction.RIGHT and self.in_lane_change):
                    speed_threshold = par.MIN_OVERTAKING_SPEED_DELTA * (1 + par.OSCILLATION_MOD)
                    time_threshold = par.MAX_OVERTAKING_TIME * (1 - par.OSCILLATION_MOD)
                # Overtaking is useful (formula and at most safety distance * factor away)
                if v_max_ot_lane - v_ot_vehicle >= speed_threshold \
                        and ot_time <= time_threshold \
                        and (ot_vehicle[1] < par.FACTOR_SAFETY_DISTANCE * self.safety_dist_platoon()
                        or self.direction == Direction.RIGHT):  # noqa 503
                    overtake = True
                else:
                    overtake = False
                # Return if a vehicle is found that could be overtaken
                if overtake:
                    return overtake, v_max_ot_lane, v_ot_vehicle, ot_time
                # Save values
                else:
                    v_ot_vehicle_all.append(round(v_ot_vehicle, 2))
                    ot_time_all.append(round(ot_time, 2))
        return False, v_max_ot_lane, v_ot_vehicle_all, ot_time_all

    def vehicle_ahead(self):
        """
        State "vehicle ahead"
        """
        vehicles_ahead = util.get_neighbor(self.v_id, direction=Direction.SAME, front=True, select='all')
        if vehicles_ahead is None or len(vehicles_ahead) == 0:
            self.debug_print('Vehicle ahead is out of range')
            # Overtaking not necessary anymore
            # print("Resetting overtaking time")
            self.ot_started = -1
            self.set_state(State.IDLE)
        else:
            if self.timer == 0:
                overtake, v_max_ot_lane, v_ot_vehicle, ot_time = self.should_overtake(vehicles_ahead)
                if overtake:
                    self.debug_print(f'Max speed on left lane: {v_max_ot_lane}, '
                                     f'speed of vehicle(s) ahead: {v_ot_vehicle}')
                    self.debug_print(f'Time needed for overtaking: {ot_time}')
                    self.debug_print(f'Overtaking of {vehicles_ahead[0]} useful')
                    # Save predicted overtaking time
                    self.ot_predicted = ot_time
                    # Save start time of overtaking decision making
                    if self.direction == Direction.LEFT and self.ot_started == -1:
                        self.ot_started = util.get_time()
                        print()
                        print("wants to overtake at", self.ot_started)
                    self.set_state(State.ASSERT_OWN_AREAS)

    def passing(self):
        """
        State: "passing"
        """
        # Save end time of overtaking
        if self.ot_started != -1:
            print()
            print("lane change complete at", util.get_time(), " - duration", util.get_time() - self.ot_started)
            self.rec_data[Key.OT_TIME].append(util.get_time() - self.ot_started)
            self.ot_started = -1
        # No timer running
        if self.timer == 0:
            change_back = False
            vehicles_fr = util.get_neighbor(self.v_id, Direction.RIGHT, front=True, select='all')
            if vehicles_fr is None or len(vehicles_fr) == 0:
                self.debug_print('FR area is free')
                change_back = True
            else:
                overtake, v_max_ot_lane, v_ot_vehicle, ot_time = self.should_overtake(vehicles_fr)
                if not overtake:
                    self.debug_print(f'Max speed on current lane: {v_max_ot_lane}, '
                                     f'speed of FR vehicle(s): {v_ot_vehicle}')
                    self.debug_print(f'Time needed for overtaking: {ot_time}')
                    self.debug_print(f'Overtaking of {vehicles_fr[0]} not useful ')
                    change_back = True
            # Change back to original lane
            if change_back:
                assert self.direction == Direction.RIGHT
                self.debug_print('Lane change back useful')
                self.set_state(State.ASSERT_OWN_AREAS)

    def assert_own_areas(self):
        """
        State "assert alpha and beta areas"

        Assert FL and RL areas (Direction.LEFT) or assert FR and RR areas (Direction.RIGHT)"
        """
        if self.alpha_area_free() and self.beta_area_free():
            self.debug_print(f'{self.direction.short()} area is free')
            self.set_state(State.REQUEST_SENSOR_DATA)
        else:
            self.debug_print(f'{self.direction.short()} area is occupied')
            self.set_state(State.LANE_CHANGE_ABORTED)

    def request_sensor_data(self):
        """
        State "request sensor data"
        """
        # Send messages to followers
        for f in self.follower:
            self.send_message(f, Message.REQ_SENSOR_DATA, {'direction': self.direction})
            self.waiting_for_reply.append(f)
        self.set_timer()
        self.set_state(State.WAIT_FOR_RESPONSES)

    def wait_for_responses(self):
        """
        State "wait for responses" (sensor data of followers)
        """
        # Timeout
        if self.timer == 0:
            # Leader is not waiting for messages anymore
            self.waiting_for_reply.clear()
            self.debug_print(f'Timer expired in state {self.state}')
            self.set_state(State.LANE_CHANGE_ABORTED)
            return
        # check for new messages
        messages = self.get_messages_by_type(Message.RESP_SENSOR_DATA)
        while len(messages) > 0:
            msg = messages.pop(0)
            self.safety_data.append(msg['data'])
            try:
                self.waiting_for_reply.remove(msg['src'])
            # Could happen when message delivery is unreliable
            except ValueError:
                self.debug_print('not waiting for this message')
        # All messages from followers received
        if len(self.waiting_for_reply) == 0:
            self.debug_print('All necessary messages received')
            self.set_state(State.ASSERT_MANEUVER_AREA)

    def assert_maneuver_area(self):
        """
        State "assert maneuver area"
        """
        # Assess safety of overtaking maneuver (based on followers data)
        is_safe = True
        while len(self.safety_data) > 0:
            data = self.safety_data.pop(0)
            if data['own-area-free'] is False:
                is_safe = False
        # Overtaking is safe
        if is_safe:
            self.debug_print('Lane change is safe')
            self.set_state(State.LANE_CHANGE_SAFE)
        # Overtaking is unsafe
        else:
            self.debug_print('Lane change is unsafe')
            self.set_state(State.LANE_CHANGE_ABORTED)

    def lane_change_aborted(self):
        """
        State "lane change aborted"
        """
        self.in_new_lane = False
        if self.direction == Direction.LEFT:
            self.ot_predicted = 0
            self.set_backoff_timer()
            self.set_state(State.VEHICLE_AHEAD)
        else:
            self.set_timer()
            self.set_state(State.PASSING)

    def lane_change_safe(self):
        """
        State "overtaking safe"
        """
        # Reset back-off
        self.backoff = par.MIN_BACKOFF
        # Send begin of lane change to followers
        assert len(self.waiting_for_reply) == 0
        for f in self.follower:
            self.send_message(f, Message.BEGIN_LANE_CHANGE, {'direction': self.direction})
            self.waiting_for_reply.append(f)
        self.begin_lane_change = True
        self.set_state(State.CHANGING_LANE)

    def handle_abort(self):
        """
        Helper method for switching to the abort state.
        """
        # Do not wait for replies anymore
        self.waiting_for_reply.clear()
        self.debug_print(f'ABORTING: {self.abort_message}')
        self.set_state(State.ABORT)

    def changing_lane(self):
        """
        State "changing lane"
        """
        # Check for 'abort' messages
        messages = self.get_messages_by_type(Message.ABORT)
        if len(messages) > 0:
            self.begin_change_back = True
            self.abort_message = f"Abort from '{messages[0]['src']}' received"
            self.handle_abort()
            return
        super().changing_lane()
        # Receive 'lane change complete' messages
        messages = self.get_messages_by_type(Message.LANE_CHANGE_COMPLETE)
        while len(messages) > 0:
            msg = messages.pop(0)
            self.waiting_for_reply.remove(msg['src'])
        # Managing possible hazards during a lane change
        if self.in_lane_change:
            # Slower vehicle not passable anymore
            if self.direction == Direction.LEFT:
                # Platoon already in new lane
                if self.in_new_lane:
                    ot_vehicles = util.get_neighbor(self.v_id, Direction.RIGHT, front=True, select='all')
                # Platoon still mostly in old lane
                else:
                    ot_vehicles = util.get_neighbor(self.v_id, direction=Direction.SAME, front=True, select='all')
                # Should not overtake (anymore)
                if not self.should_overtake(ot_vehicles)[0]:
                    self.detection_threshold += 1
                    # Overtaken vehicle is not detected in multiple simulation steps
                    if self.detection_threshold > par.MAX_NOT_DETECTED:
                        self.begin_change_back = True
                        if ot_vehicles is None:
                            self.abort_message = 'Slower vehicle out of range'
                        else:
                            self.abort_message = f'Overtaking of {ot_vehicles[0]} not useful or possible anymore'
                            self.debug_print(f'Debug: v_FL={self.should_overtake(ot_vehicles)[1]}, '
                                             f'v_F={self.should_overtake(ot_vehicles)[2]}, '
                                             f't_o={self.should_overtake(ot_vehicles)[3]}')
                else:
                    self.detection_threshold = 0
            # Slower vehicle appeared during lane change right
            else:
                # Platoon already in new lane
                if self.in_new_lane:
                    ot_vehicles = util.get_neighbor(self.v_id, direction=Direction.SAME, front=True, select='all')
                # Platoon still mostly in old lane
                else:
                    ot_vehicles = util.get_neighbor(self.v_id, Direction.RIGHT, front=True, select='all')
                # Should overtake this vehicle, too
                if self.should_overtake(ot_vehicles)[0]:
                    self.detection_threshold += 1
                    # Overtaken vehicle is not detected in multiple simulation steps
                    if self.detection_threshold > par.MAX_NOT_DETECTED:
                        self.begin_change_back = True
                        self.abort_message = f'Overtaking of {ot_vehicles[0]} useful and possible'
                        self.debug_print(f'Debug: v_FL={self.should_overtake(ot_vehicles)[1]}, '
                                         f'v_F={self.should_overtake(ot_vehicles)[2]}, '
                                         f't_o={self.should_overtake(ot_vehicles)[3]}')
                else:
                    self.detection_threshold = 0
            # Abort lane change
            if self.begin_change_back:
                self.handle_abort()
        # Driving in new lane and all responses from followers received
        elif not self.in_lane_change and len(self.waiting_for_reply) == 0:
            self.debug_print('All necessary messages received')
            self.debug_print('Lane change of platoon completed')
            self.set_state(State.LANE_CHANGE_COMPLETE)

    def lane_change_complete(self):
        """
        State "lane change complete"
        """
        super().lane_change_complete()
        self.safety_data.clear()
        self.timer = 0
        # Inform followers about completeness of lane change maneuver
        for f in self.follower:
            self.send_message(f, Message.LANE_CHANGE_COMPLETE)
        if self.direction == Direction.RIGHT:
            self.debug_print('-------')
            self.debug_print('Passing')
            self.debug_print('-------')
            self.set_state(State.PASSING)
        else:
            self.ot_predicted = 0
            self.debug_print('Platoon overtaking complete')
            self.set_state(State.OVERTAKING_COMPLETE)

    def overtaking_complete(self):
        """
        State "overtaking complete"
        """
        self.debug_print('Going back to idle')
        self.set_state(State.IDLE)

    def abort(self):
        """
        State "abort"
        """
        # Send 'abort' messages to followers
        for f in self.follower:
            self.send_message(f, Message.ABORT, {'direction': self.direction})
            self.waiting_for_reply.append(f)
        self.set_state(State.CHANGING_BACK)

    def changing_back(self):
        """
        State "changing back"
        """
        super().changing_back()
        # Receive 'abort complete' messages
        messages = self.get_messages_by_type(Message.ABORT_COMPLETE)
        while len(messages) > 0:
            msg = messages.pop(0)
            self.waiting_for_reply.remove(msg['src'])
        # Changing back done and all responses from followers received
        if not self.in_change_back and len(self.waiting_for_reply) == 0:
            self.debug_print('All necessary messages received')
            self.debug_print('Platoon returned to original lane')
            self.set_state(State.LANE_CHANGE_ABORTED)
