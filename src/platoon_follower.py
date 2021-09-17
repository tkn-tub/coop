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

from c2x import C2X
from message import Message
from platoon_vehicle import PlatoonVehicle
from state import State


class PlatoonFollower(PlatoonVehicle):
    """
    Class that handles a platoon follower.
    """

    def __init__(self, v_id: str, c2x: C2X):
        super().__init__(v_id, c2x)
        self.leader = None
        self.front = None

    def set_leader(self, leader_id):
        self.leader = leader_id

    def next_step(self):
        """
        Executes one step of the lane change algorithm.
        """
        super().next_step()

        # State: idle
        if self.state == State.IDLE:
            self.idle()

        # State: sensor data retrieval
        elif self.state == State.ASSERT_OWN_AREAS:
            self.assert_own_areas()

        # State: wait for decision
        elif self.state == State.WAIT_FOR_DECISION:
            self.wait_for_decision()

        # State: changing lane
        elif self.state == State.CHANGING_LANE:
            self.changing_lane()

        # State: in overtaking lane
        elif self.state == State.IN_OVERTAKING_LANE:
            self.in_overtaking_lane()

        # State: lane changed
        elif self.state == State.LANE_CHANGED:
            self.lane_changed()

        # State: lane change complete
        elif self.state == State.LANE_CHANGE_COMPLETE:
            self.lane_change_complete()

        elif self.state == State.ABORT:
            self.abort()

        elif self.state == State.CHANGING_BACK:
            self.changing_back()

        elif self.state == State.IN_ORIGINAL_LANE:
            self.in_original_lane()

    def idle(self):
        """
        State "idle"
        """
        # Check for new messages
        messages = self.get_messages_by_type(Message.REQ_SENSOR_DATA)
        if len(messages) > 0:
            self.direction = messages[0]['data']['direction']
            self.set_state(State.ASSERT_OWN_AREAS)
        # Check for 'abort' messages (can happen when message reception is unreliable)
        messages = self.get_messages_by_type(Message.ABORT)
        if len(messages) > 0:
            self.send_message(self.leader, Message.ABORT_COMPLETE)

    def assert_own_areas(self):
        """
        State "assert own areas"
        """
        self.send_message(self.leader, Message.RESP_SENSOR_DATA,
                          {'own-area-free': self.alpha_area_free() and self.beta_area_free()})
        self.set_timer()
        self.set_state(State.WAIT_FOR_DECISION)

    def wait_for_decision(self):
        """
        State "wait for decision"
        """
        # Timer expired
        if self.timer == 0:
            self.debug_print(f'Timer expired in state {self.state}')
            self.set_state(State.IDLE)
            return
        # Check for 'begin lane change' messages
        messages = self.get_messages_by_type(Message.BEGIN_LANE_CHANGE)
        if len(messages) > 0:
            self.begin_lane_change = True
            self.set_state(State.CHANGING_LANE)

    def changing_lane(self):
        """
        State "changing lane"
        """
        # Check for 'abort' messages
        messages = self.get_messages_by_type(Message.ABORT)
        if len(messages) > 0:
            self.begin_change_back = True
            self.set_state(State.CHANGING_BACK)
            return
        # Standard lane change functionality
        super().changing_lane()
        # Lane change complete
        if not self.in_lane_change:
            self.debug_print('Actual lane change completed')
            self.set_state(State.LANE_CHANGED)
        # During lane change
        else:
            # Lane change has to be aborted
            if self.begin_change_back:
                self.debug_print(f'ABORTING: {self.abort_message}')
                self.set_state(State.ABORT)

    def handle_abort_and_lc_complete_messages(self):
        """
        Handles abort and lane change complete messages
        """
        # Check for 'abort' messages
        messages = self.get_messages_by_type(Message.ABORT)
        if len(messages) > 0:
            self.begin_change_back = True
            self.set_state(State.CHANGING_BACK)
        # Check for 'lane change complete' messages
        messages = self.get_messages_by_type(Message.LANE_CHANGE_COMPLETE)
        if len(messages) > 0:
            self.debug_print('Leader confirmed completeness of lane change maneuver')
            self.set_state(State.LANE_CHANGE_COMPLETE)

    def lane_changed(self):
        """
        State "lane changed"
        """
        self.handle_abort_and_lc_complete_messages()
        if self.state == State.LANE_CHANGED:
            self.send_message(self.leader, Message.LANE_CHANGE_COMPLETE)
            self.set_state(State.IN_OVERTAKING_LANE)

    def in_overtaking_lane(self):
        """
        State "in overtaking lane" (belongs to "lane changed" in FSM)
        """
        self.handle_abort_and_lc_complete_messages()

    def lane_change_complete(self):
        """
        State: "lane change (maneuver) complete" (not part of FSM, only for clean-up reasons)
        """
        super().lane_change_complete()
        self.debug_print('Going back to idle')
        self.set_state(State.IDLE)

    def abort(self):
        """
        State: "abort"
        """
        # Send abort message to leader
        self.send_message(self.leader, Message.ABORT)
        self.set_state(State.CHANGING_BACK)

    def changing_back(self):
        """
        State: "changing back"
        """
        super().changing_back()
        # Lane change back complete
        if not self.in_change_back:
            self.debug_print('Lane change back complete')
            self.set_state(State.IN_ORIGINAL_LANE)

    def in_original_lane(self):
        """
        State: "in original lane"
        """
        self.in_new_lane = False
        self.send_message(self.leader, Message.ABORT_COMPLETE)
        self.set_state(State.IDLE)
