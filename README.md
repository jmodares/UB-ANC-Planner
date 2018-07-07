Copyright © 2016 - 2018 Jalil Modares

This program was part of my Ph.D. Dissertation research in the Department of Electrical Engineering at the University at Buffalo. I worked in UB's Multimedia Communications and Systems Laboratory with my Ph.D. adviser, [Prof. Nicholas Mastronarde](http://www.eng.buffalo.edu/~nmastron).

If you use this program for your work/research, please cite: [J. Modares, F. Ghanei, N. Mastronarde and K. Dantu, "UB-ANC Planner: Energy Efficient Coverage Path Planning with Multiple Drones"](https://doi.org/10.1109/ICRA.2017.7989732).

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program. If not, see <http://www.gnu.org/licenses/>.

# UB-ANC Planner
## A Multi-Agent Energy Efficent Coverage Path Planner
Utilizing the UB-ANC Drone and [UB-ANC Emulator](https://github.com/jmodares/UB-ANC-Emulator) projects, we built an application for airborne networks, called [UB-ANC Planner](https://github.com/jmodares/UB-ANC-Planner). It considers the problem of covering an arbitrary area containing obstacles using multiple drones, i.e., the so-called **Coverage Path Planning (CPP)** problem. The goal of the CPP problem is to find paths for each drone such that the entire area is covered. However, a major limitation in such deployments is drone flight time. To most efficiently use a swarm, we propose to minimize the maximum energy consumption among all drones' flight paths. We perform measurements to understand energy consumption of a drone. Using these measurements, we formulate an **Energy Efficient Coverage Path Planning (EECPP)** problem. We solve this problem in two steps: a load-balanced allocation of the given area to individual drones, and a **Minimum Energy Path Planning (MEPP)** problem for each drone. We conjecture that the MEPP is NP-hard as it is similar to the **Traveling Salesman Problem (TSP)**. We propose an adaptation of the well-known **Lin-Kernighan Heuristic (LKH)** for the TSP to efficiently solve the problem. We compare our solution to the recently proposed depth-limited search with back tracking algorithm, the optimal solution, and rastering as a baseline. Results show that our algorithm is more computationally efficient and provides more energy-efficient solutions compared to the other heuristics.

## Build

The current version of the UB-ANC Planner uses [IBM ILOG CPLEX Optimization Studio 12.63](https://www.ibm.com/products/ilog-cplex-optimization-studio) and [Qt](https://www.qt.io/) as its main libraries. The build process explained here is targeted for Linux (Debian compatible) platforms. We recommend using [Ubuntu 16.04](http://releases.ubuntu.com/16.04/). Before building the UB-ANC Planner, `cplex.pri` file should be changed to point to the correct location of CPLEX library. Next run the following commands to setup your system:
```
sudo apt-get update && sudo apt-get upgrade
sudo apt-get install qt5-default qtbase5-dev \
    qtdeclarative5-dev qtpositioning5-dev \
    qtlocation5-dev libqt5svg5-dev libqt5serialport5-dev
```

Then, use `qmake` to build the planner:

```
cd ~
mkdir ub-anc && cd ub-anc
git clone https://github.com/jmodares/UB-ANC-Planner
mkdir build-planner && cd build-planner
qmake ../UB-ANC-Planner
make -j4
```

## Run
There are different command line options that need to be set, the important one is `-f` or `--file` which specify the mission file for the planner. Use `-h` or `--help` to see all options and their descriptions.
