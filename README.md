# CoOP: V2V-based Cooperative Overtaking for Platoons on Freeways

This repository includes all necessary files for simulating the cooperative overtaking algorithm for platoons on freeways as proposed in

> Martin Strunz, Julian Heinovski and Falko Dressler, "CoOP: V2V-based Cooperative Overtaking for Platoons on Freeways," Proceedings of 24th IEEE International Conference on Intelligent Transportation Systems (ITSC 2021), Indianapolis, IN, September 2021.

[SUMO](https://www.eclipse.org/sumo/) (Simulation of Urban MObility) is used as simulation environment.
The [Plexe APIs for Python](https://github.com/michele-segata/plexe-pyapi) provide general platooning functionality.

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

### Prerequisites

The following software has to be installed:

* [Python 3](https://www.python.org/) - Version 3.4 or later required
* [SUMO](https://www.eclipse.org/sumo/) - Simulation of Urban MObility - Version 1.7.0 or higher required
* [Plexe APIs for Python](https://github.com/michele-segata/plexe-pyapi) - platooning functionality for SUMO
* [pytest](https://docs.pytest.org/en/stable/) - to execute the included test cases
* [pytest-mock](https://pypi.org/project/pytest-mock/) - provides a mocker fixture

### Installing

Just clone the repository.

### Parameters

All relevant parameters of the algorithm can be defined in [parameter.py](src/parameter.py).
The most important for the first start are:

```
SHOW_GUI = True
SHOW_SIMUWATCH = True
DEBUG = True
```

`SHOW_GUI` defines whether the GUI of SUMO is turned on or off.
`SHOW_SIMUWATCH` defines whether a small additional information window is shown. It is not required to run the algorithm.
If `DEBUG = True`, a detailed log is printed to stdout.


## Executing the Algorithm

The algorithm comes with a lot of test cases that can be found in the [tests](tests) folder.
Unit tests for the state transfers of the finite state machines can be found in the [tests/test_state_transfer](tests/test_state_transfer) folder.
These tests are meant to show, that the algorithm is correctly implemented, and are therefore of secondary importance for the user.
The second type of test cases are tests for different traffic situations that can be found in the [tests/test_traffic](tests/test_traffic) folder.
Message delivery is reliable in most test cases by setting the parameter `MEAN_STEP_DURATION_MSG_DELIVERY = 0`.
All test cases are based on German law and therefore set the parameters `MIN_OVERTAKING_SPEED_DELTA = 2.7` and `MAX_OVERTAKING_TIME = 45`.

### Types of Test Cases

* `tests/test_state_transfer/` - Tests for the state transfers of the algorithm.
* `tests/test_traffic/` - Tests for different traffic scenarios:
    * `./test_basic.py` - Basic test cases
    * `./test_a.py` - Decision phase test cases
    * `./test_b.py` - Lane change phase test cases

You can run the tests using `pytest` from the root directory of this repository:
```
env PYTHONPATH=$(pwd)/src pytest -v tests/
```
Note that here the `src` directory is added to `PYTHONPATH` for just this command.

## Using the Algorithm in Own Test Cases

Use the following guideline to build a platoon that utilizes the overtaking algorithm.

### Initialize TraCI and Plexe
```
traci.start(sumo_cmd)
plexe = Plexe()
traci.addStepListener(plexe)
```

`sumo_cmd` contains the relevant SUMO parameters. Please see SUMO and Plexe documentations for further information.

### Build a Platoon
```
c2x = C2X(seed=7005)
platoon = Platoon(plexe, c2x)
platoon.build(n=4, speed=30.55, vType='PlatoonCar', route='freeway', pos=20, lane=0)
```
This will build a platoon with `4` vehicles (including the leader) of vehicle type
`PlatoonCar` with desired speed `30.55` m/s on the route `freeway` with starting position `20m` in lane `0`.
The vehicle type as well as the route have to be defined in the according `.rou.xml` file of SUMO.

Call `platoon.overtaking_step()` in each simulation step to execute the overtaking algorithm.

## Citing

If you are working with `CoOP`, please cite the following paper:

> Martin Strunz, Julian Heinovski and Falko Dressler, "CoOP: V2V-based Cooperative Overtaking for Platoons on Freeways," Proceedings of 24th IEEE International Conference on Intelligent Transportation Systems (ITSC 2021), Indianapolis, IN, September 2021.

```bibtex
@inproceedings{strunz2021coop,
    author = {Strunz, Martin and Heinovski, Julian and Dressler, Falko},
    doi = {10.1109/ITSC48978.2021.9565122},
    title = {{CoOP: V2V-based Cooperative Overtaking for Platoons on Freeways}},
    publisher = {IEEE},
    isbn = {978-1-7281-9142-3},
    address = {Indianapolis, IN},
    booktitle = {24th IEEE International Conference on Intelligent Transportation Systems (ITSC 2021)},
    month = {9},
    year = {2021},
}
```

PDF and Details can be found at [https://www2.tkn.tu-berlin.de/bib/strunz2021coop/](https://www2.tkn.tu-berlin.de/bib/strunz2021coop/).

## License
```
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
```
