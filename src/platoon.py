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

"""
Functions for platooning.
"""
import parameter as par
import util
from c2x import C2X
from key import Key
from platoon_follower import PlatoonFollower
from platoon_leader import PlatoonLeader
from state import State
from util import add_vehicle


class Platoon:
    """
    Class to build and manage a platoon.
    """

    def __init__(self, plexe, c2x: C2X):
        self.plexe = plexe
        self.leader = None
        self.follower = []
        self.wifi = c2x
        self.did_overtake = False

    def build(self, n=5, speed=par.SPEED, vType='PlatoonCar', route=par.PLATOON_ROUTE, pos=20, lane=0):
        """
        Builds a platoon of n vehicles. SUMO-GUI will track the platoon leader.

        :param route: SUMO route
        :param pos: longitudinal position where the platoon should start on the lane
        :param plexe: Plexe
        :param n: number of platoon members (including leader)
        :param speed: desired platoon speed
        :param vType: vType of platoon members
        """
        par.PLATOON_MEMBERS = n
        leader_pos = 0
        for i in range(n):
            v_id = f'{par.ID_PRE}_{i}'
            length = util.get_vtype_length(vType)
            # add and configure platooning vehicle
            add_vehicle(self.plexe, v_id, route,
                        pos=(n - i + 1) * (par.INTER_VEHICLE_DISTANCE + length) + pos, lane=lane,
                        speed=speed, vtype=vType, color=(0, 255 - i * 10, 255 - i * 10, 255))
            self.plexe.set_path_cacc_parameters(v_id, par.INTER_VEHICLE_DISTANCE, par.XI, par.OMEGA_N, par.C1)
            self.plexe.set_cc_desired_speed(v_id, speed)
            self.plexe.set_acc_headway_time(v_id, par.ACC_HEADWAY)
            # keep platoon members in specified lane
            self.plexe.set_fixed_lane(v_id, lane, safe=False)
            # all speed checks off (https://sumo.dlr.de/docs/TraCI/Change_Vehicle_State.html#speed_mode_0xb3)
            util.set_speed_mode(v_id, 0)
            util.set_lane_change_mode(v_id, 0b0000000000)
            if i == 0:
                self.plexe.set_active_controller(v_id, par.PLEXE_LEADER_CONTROLLER)
                if par.PLEXE_OVERTAKING_ON:
                    self.plexe.enable_auto_lane_changing(v_id, True)
            else:
                self.plexe.set_active_controller(v_id, par.PLEXE_FOLLOWER_CONTROLLER)
                self.plexe.enable_auto_feed(v_id, True, par.LEADER, f'{par.ID_PRE}_{i - 1}')
                self.plexe.add_member(par.LEADER, v_id, i)
            # add platooning vehicle to Platoon()
            if not par.PLEXE_OVERTAKING_ON:
                if i == 0:
                    leader_pos = (n - i + 1) * (par.INTER_VEHICLE_DISTANCE + length) + pos
                    self.leader = PlatoonLeader(v_id, self.wifi)
                    self.leader.v_desired = speed
                else:
                    new_follower = PlatoonFollower(v_id, self.wifi)
                    new_follower.front = f'{par.ID_PRE}_{i - 1}'
                    new_follower.leader = par.LEADER
                    self.follower.append(new_follower)
                    # tell leader about follower
                    self.leader.follower.append(v_id)

        # track leader
        if par.SHOW_GUI:
            util.track_vehicle("View #0", par.LEADER)

        # Return leaders position
        return leader_pos

    def overtaking_step(self):
        """
        Executes the next step of the cooperative overtaking algorithm for each platoon member individually.
        """
        # Overtaking complete
        if self.leader.state == State.OVERTAKING_COMPLETE:
            self.did_overtake = True
        # Execute next step of the overtaking algorithm
        self.leader.next_step()
        for f in self.follower:
            f.next_step()

    def get_data_for_key(self, key: Key):
        """
        Returns the recorded data for key of each platoon member.

        :param key: key of data that was recorded while driving
        :return: dict with recorded data: {'leader': data, 'follower': [data_follower_1, data_follower_2, ...]}
        """
        result = {self.leader.v_id: self.leader.rec_data[key]}
        for f in self.follower:
            result[f.v_id] = f.rec_data[key]
        return result

    def print(self):
        """
        Print platoon formation for debug purposes.
        """
        print('Leader:')
        self.leader.print()
        print('Follower:')
        for follow in self.follower:
            follow.print()
