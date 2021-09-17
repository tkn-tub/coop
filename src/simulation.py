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
Different platooning simulations
"""
import os
import sys

import traci
from plexe import Plexe

import parameter as par
import util
from c2x import C2X
from key import Key
from platoon import Platoon
from simu_watch import SimuWatch


class Simulation:
    """
    Runs a simulation with a given platoon and records relevant data.
    """

    def __init__(self, sumocfg=par.SUMO_TEST_CFG, scenario='None', use_plexe=True, seed=par.RND_SEED):
        # Setting up SUMO, TraCI and Plexe
        if not par.SUMO_HOME:
            sys.exit("Please declare variable 'SUMO_HOME'")
        sumo_prog = os.path.join(par.SUMO_HOME, 'bin', 'sumo')
        # Print SUMO version
        os.system(sumo_prog)
        if par.SHOW_GUI:
            sumo_prog = os.path.join(par.SUMO_HOME, 'bin', 'sumo-gui')
        sumo_cmd = [sumo_prog] + par.SUMO_PARAMS + ["-c", sumocfg]
        traci.start(sumo_cmd, numRetries=1)
        print(f'TraCI version: {traci.getVersion()}')
        self.plexe = Plexe()
        if use_plexe:
            print('Adding Plexe to step listener')
            traci.addStepListener(self.plexe)

        # Initialize SimuWatch
        self.sw = None

        # Initialize messaging system
        self.c2x = C2X(seed)

        self.step = 0
        self.results = dict({'scenario': scenario,
                             'v_rec_id': 'None',
                             'rec_per_sec': par.REC_PER_SEC,
                             'collision': False,
                             'minGap': [float('inf'), -1, 'None']})

    def run(self, platoon: Platoon, v_rec_id=par.LEADER, test_commands: dict = None,
            t_platoon=-1, n_platoon=3, pos_platoon=130, lane_platoon=0, record_lane_change=False) -> dict:
        """
        Executes the simulation with the defined platoon and records relevant data.

        :param platoon: platoon formation as created by Platoon()
        :param v_rec_id: id of the vehicle whose data is to be recorded
        :param test_commands: Commands that are relevant for execution under pytest
        :param record_lane_change: Toggles whether the lane change time is recorded
        :return: dictionary with recorded data
        """
        self.results['v_rec_id'] = v_rec_id
        self.results[Key.OT_TIME] = []

        # Start SimuWatch
        if par.SHOW_SIMUWATCH:
            self.sw = SimuWatch(platoon)

        # Data for overtaking intention of single vehicle
        change_lane_info = 0
        ot_start = -1

        while self.step <= t_platoon or traci.vehicle.getDistance(par.LEADER) < par.DISTANCE_TO_DRIVE:

            if self.step % 10000 == 0:
                print()
                if self.step >= t_platoon:
                    print(f'* distance driven: {int(traci.vehicle.getDistance(par.LEADER))}')
                else:
                    print(f'* simulation time: {int(self.step * par.STEP_LENGTH)}')

            # Insert platoon if not already done
            if self.step == t_platoon:
                if n_platoon > 1:
                    platoon.build(n=n_platoon, pos=pos_platoon, lane=lane_platoon)
                else:
                    # Insert single car with length of a whole platoon
                    # Position of platoon with 4 cars: 114.7 <- -> 148.50
                    util.add_vehicle(self.plexe, par.LEADER, par.PLATOON_ROUTE, 148.5, 0,
                                     par.SPEED, 'LongCar')
                    util.set_speed(par.LEADER, par.SPEED)
                    if par.SHOW_GUI:
                        util.track_vehicle("View #0", par.LEADER)

            traci.simulationStep()

            # Record additional lane change data if required
            if self.step >= t_platoon and record_lane_change:
                change_lane_info_last_step = change_lane_info
                change_lane_info = traci.vehicle.getLaneChangeStatePretty(par.LEADER, 1)[0]
                # Lane change state has changed
                if change_lane_info_last_step != change_lane_info:
                    # print("New state at ", util.get_time(), ": ", change_lane_info)
                    # Wants to change to the left
                    if 'left' in change_lane_info:
                        if 'sublane' not in change_lane_info:
                            # First intention to change to the left
                            if ot_start == -1:
                                print()
                                print("* Wants to change to the left at ", util.get_time())
                                ot_start = util.get_time()
                    # No overtaking intention anymore
                    if len(change_lane_info) == 0:
                        if ot_start != -1 and traci.vehicle.getLaneIndex(par.LEADER) == 1:
                            print()
                            print(
                                "* Changed to the left at ", util.get_time(),
                                " - Duration: ", util.get_time() - ot_start
                            )
                            self.results[Key.OT_TIME].append(util.get_time() - ot_start)
                        # Reset overtaking start time
                        ot_start = -1

            # Only execute algorithm specific code if platoon length > 1
            if n_platoon > 1:

                # Execute a step of the overtaking algorithm
                if self.step >= t_platoon and par.OVERTAKING_ALGORITHM_ON:
                    platoon.overtaking_step()

                # Eval safety
                if t_platoon > -1:
                    self.collision_detection()
                if par.SHOW_SIMUWATCH and self.step >= t_platoon:
                    # Output information in SimuWatch
                    self.sw.leader(step=self.step)
                    self.sw.update()

                # Execute additional test commands (when run with pytest)
                if test_commands is not None:
                    # A command for this simulation step was defined
                    if self.step in test_commands.keys():
                        cmd = test_commands[self.step]
                        # Add a vehicle without collision checks
                        if cmd['cmd'] == 'add_vehicle':
                            util.add_vehicle(self.plexe, cmd['v_id'], par.PLATOON_ROUTE, 0, 0,
                                             cmd['speed'], cmd['vtype'])
                            util.move_vehicle(cmd['v_id'], cmd['lane_id'], cmd['pos'])
                            util.set_speed(cmd['v_id'], cmd['speed'])
                            print(
                                f"({self.step}) TEST-COMMAND: Adding vehicle '{cmd['v_id']}' with speed {cmd['speed']}")
                        # Remove a vehicle
                        elif cmd['cmd'] == 'remove_vehicle':
                            print(f"({self.step}) TEST-COMMAND: Removing vehicle '{cmd['v_id']}'")
                            util.remove_vehicle(cmd['v_id'])
                        # Change speed of a vehicle
                        elif cmd['cmd'] == 'new_speed':
                            print(f"({self.step}) TEST-COMMAND: Setting speed of '{cmd['v_id']}' to {cmd['speed']}")
                            util.set_speed(cmd['v_id'], cmd['speed'])
                        # Change sub-lane of a vehicle
                        elif cmd['cmd'] == 'change_lane':
                            traci.vehicle.setLaneChangeMode(cmd['v_id'], 0b000000000000)
                            lane_width = util.get_lane_width(cmd['v_id'])
                            util.change_sublane(cmd['v_id'], lane_width * cmd['direction'])
                        elif cmd['cmd'] == 'new_color':
                            print(f"({self.step}) TEST-COMMAND: Setting color of '{cmd['v_id']}' to {cmd['color']}")
                            util.set_color(cmd['v_id'], cmd['color'])

                # Completing the overtaking maneuver concludes the simulation
                if par.EXIT_ON_OVERTAKING and platoon.did_overtake:
                    break

            self.step += 1

        # Safe additional platoon data
        if not par.PLEXE_OVERTAKING_ON and n_platoon > 1:
            self.results[Key.STATES_VISITED] = platoon.get_data_for_key(Key.STATES_VISITED)
            self.results[Key.LANES_VISITED] = platoon.get_data_for_key(Key.LANES_VISITED)
            self.results[Key.SPEED_MIN] = platoon.get_data_for_key(Key.SPEED_MIN)
            self.results[Key.SPEED_MAX] = platoon.get_data_for_key(Key.SPEED_MAX)
            self.results[Key.OT_TIME] = platoon.get_data_for_key(Key.OT_TIME)
            # Final position of vehicles
            self.results[Key.POSITION_STOP] = {}
            # Distance driven
            self.results[Key.DISTANCE_DRIVEN] = {}
            for v in traci.vehicle.getIDList():
                self.results[Key.POSITION_STOP][v] = traci.vehicle.getPosition(v)[0]
                self.results[Key.DISTANCE_DRIVEN][v] = traci.vehicle.getDistance(v)

        # Clean-up
        self.cleanup()
        return self.results

    def cleanup(self):
        traci.close()
        if par.SHOW_SIMUWATCH:
            self.sw.quit()

    def collision_detection(self):
        """
        Checks if there was a collision in the previous step. If that is the case it is recorded in self.results.
        """
        crashes = traci.simulation.getCollidingVehiclesNumber()
        if crashes > 0:
            self.results['collision'] = True
            if par.SHOW_SIMUWATCH:
                self.sw.collision()
