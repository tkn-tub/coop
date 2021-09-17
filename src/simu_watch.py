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

import tkinter as tk

import parameter as par
import util
from direction import Direction
from platoon import Platoon


def update_label(label, text):
    """
    Sets the text of a given (already defined) label to the given text.
    """
    if label['text'] is not str(text):
        label['text'] = str(text)


class SimuWatch:
    """
    Simple GUI to conveniently observe simulation data.
    """
    # Leader
    LBL_LEADER = 0
    SPEED_COL = 1
    STATE_COL = 2
    DIRECTION_COL = 3
    IN_FRONT_COL = 4
    IN_REAR_COL = 5
    LEFT_LEADER_COL = 6
    LEFT_FOLLOWER_COL = 7
    RIGHT_LEADER_COL = 8
    RIGHT_FOLLOWER_COL = 9
    IN_LANE_CHANGE = 10
    IN_CHANGE_BACK = 11
    IN_NEW_LANE = 12
    # Follower
    LBL_FOLLOWER = 20
    STATE_FOLL = 21
    DIRECTION_FOLL = 22
    # Last Follower
    LBL_LAST_FOLLOWER = 30
    STATE_LAST_F = 31
    LAST_FOLL = 32
    LAST_LEFT_FOLL = 33
    # Simulation data
    LBL_SIMULATION = 40
    OVERTAKE_TIME_PREDICTED = 41
    OVERTAKE_TIME_ACTUAL = 42
    MIN_GAP_COL = 43
    COLLISION_COL = 44

    def __init__(self, platoon: Platoon):
        self.window = tk.Tk()
        self.window.title('SimuWatch v1.0')
        self.window.geometry("300x400+800+560")
        self.window.resizable(0, 0)

        self.platoon = platoon

        # Leader info
        self.lbl_leader = self.make_label('LEADER', '', self.LBL_LEADER, 'w')
        self.speed_label = self.make_label('Speed:', par.SPEED, self.SPEED_COL, 'w')
        self.state_label = self.make_label('State:', 'None', self.STATE_COL, 'w')
        self.direction_label = self.make_label('Direction:', 'None', self.DIRECTION_COL, 'w')
        # Neighbors
        self.veh_front_label = self.make_label('Vehicle in front:', 'None', self.IN_FRONT_COL, 'w')
        self.veh_rear_label = self.make_label('Vehicle in rear:', 'None', self.IN_REAR_COL, 'w')
        self.left_leader = self.make_label('Left leader:', 'None', self.LEFT_LEADER_COL, 'w')
        self.left_follower = self.make_label('Left follower:', 'None', self.LEFT_FOLLOWER_COL, 'w')
        self.right_leader = self.make_label('Right leader:', 'None', self.RIGHT_LEADER_COL, 'w')
        self.right_follower = self.make_label('Right follower:', 'None', self.RIGHT_FOLLOWER_COL, 'w')
        self.in_lane_change = self.make_label('In lane change:', 'False', self.IN_LANE_CHANGE, 'w')
        self.in_change_back = self.make_label('In change back:', 'False', self.IN_CHANGE_BACK, 'w')
        self.in_new_lane = self.make_label('In new lane:', 'False', self.IN_NEW_LANE, 'w')
        # First follower info
        self.lbl_follower = self.make_label('FIRST FOLLOWER', '', self.LBL_FOLLOWER, 'w')
        self.state_follower_label = self.make_label('State:', 'None', self.STATE_FOLL, 'w')
        self.direction_follower_label = self.make_label('Direction:', 'None', self.DIRECTION_FOLL, 'w')
        # Last follower info
        self.lbl_last_follower = self.make_label('LAST FOLLOWER', '', self.LBL_LAST_FOLLOWER, 'w')
        self.lbl_state_last = self.make_label('State:', 'None', self.STATE_LAST_F, 'w')
        self.lbl_follower_last = self.make_label('Follower:', 'None', self.LAST_FOLL, 'w')
        self.lbl_left_follower_last = self.make_label('Left follower:', 'None', self.LAST_LEFT_FOLL, 'w')

    def make_label(self, key, value, row, sticky) -> tk.Label:
        tk.Label(self.window, text=key).grid(row=row, column=0, sticky='w')
        label = tk.Label(self.window, text=value)
        label.grid(row=row, column=1, sticky=sticky)
        return label

    def update(self):
        self.window.update()

    def leader(self, step):
        # get data for labels
        l_speed = round(util.get_speed(par.LEADER), 1)
        l_state = self.platoon.leader.state
        l_direction = self.platoon.leader.direction
        f_state = self.platoon.follower[0].state
        f_direction = self.platoon.follower[0].direction
        l_slower_vehicle = util.get_leader(par.LEADER)
        if l_slower_vehicle is not None:
            l_sv_id = l_slower_vehicle[0]
            l_sv_dist = round(l_slower_vehicle[1], 2)
        else:
            l_sv_id = 'None'
            l_sv_dist = '-'
        l_rear = util.get_follower(par.LEADER)
        if l_rear is None:
            l_rear_id = 'None'
        else:
            l_rear_id = l_rear[0]
        l_left_leader = util.get_neighbor(par.LEADER, direction=Direction.LEFT, front=True)
        l_left_follower = util.get_neighbor(par.LEADER, direction=Direction.LEFT, front=False)
        l_right_leader = util.get_neighbor(par.LEADER, direction=Direction.RIGHT, front=True)
        l_right_follower = util.get_neighbor(par.LEADER, direction=Direction.RIGHT, front=False)
        lf_state = self.platoon.follower[-1].state
        lf_follower = util.get_follower(self.platoon.follower[-1].v_id)
        if lf_follower is None:
            lf_follower_id = 'None'
        else:
            lf_follower_id = lf_follower[0]
        lf_left_follower = util.get_neighbor(self.platoon.follower[-1].v_id, direction=Direction.LEFT,
                                             front=False)

        # update labels
        update_label(self.speed_label, l_speed)
        update_label(self.state_label, l_state)
        update_label(self.direction_label, l_direction)
        update_label(self.state_follower_label, f_state)
        update_label(self.direction_follower_label, f_direction)
        update_label(self.veh_front_label, f'{l_sv_id} ({l_sv_dist})')
        update_label(self.veh_rear_label, l_rear_id)
        left_follow_string = 'None'
        left_leader_string = 'None'
        right_follow_string = 'None'
        right_leader_string = 'None'
        if l_left_leader is not None:
            left_leader_string = f'{l_left_leader[0]} ({round(l_left_leader[1], 2)})'
        if l_left_follower is not None:
            left_follow_string = f'{l_left_follower[0]} ({round(l_left_follower[1], 2)})'
        if l_right_leader is not None:
            right_leader_string = f'{l_right_leader[0]} ({round(l_right_leader[1], 2)})'
        if l_right_follower is not None:
            right_follow_string = f'{l_right_follower[0]} ({round(l_right_follower[1], 2)})'
        update_label(self.left_leader, left_leader_string)
        update_label(self.left_follower, left_follow_string)
        update_label(self.right_leader, right_leader_string)
        update_label(self.right_follower, right_follow_string)
        update_label(self.in_lane_change, self.platoon.leader.in_lane_change)
        update_label(self.in_change_back, self.platoon.leader.in_change_back)
        update_label(self.in_new_lane, self.platoon.leader.in_new_lane)
        update_label(self.lbl_follower_last, lf_follower_id)
        update_label(self.lbl_state_last, lf_state)
        last_left_follower_string = 'None'
        if lf_left_follower is not None:
            last_left_follower_string = f'{lf_left_follower[0]} ({round(lf_left_follower[1], 2)})'
        update_label(self.lbl_left_follower_last, last_left_follower_string)

    def quit(self):
        self.window.quit()
