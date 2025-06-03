# Path Planning Algorithms for Autonomous Mobile Robots in Warehouse Environments

## Overview

This project investigates and compares various path planning algorithms for autonomous mobile robots (AMRs) operating in warehouse environments. The goal is to explore how different algorithms respond to static and dynamic obstacles and evaluate their efficiency in terms of path optimality and adaptability.

## Algorithms Implemented

### A* (A-star)
A classic graph-based search algorithm that combines the advantages of Dijkstra’s algorithm and heuristic methods. It is designed to find the shortest path between two nodes efficiently.

### D* (Dynamic A-star)
An enhancement of A* that dynamically updates the cost of edges in real-time, enabling adaptive path re-planning when new obstacles are detected.

### D* Lite
A further optimization of the D* algorithm. It minimizes computational overhead by updating only the affected portion of the path after environmental changes.

### RRT (Rapidly-exploring Random Trees)
A sampling-based algorithm that incrementally builds a space-filling tree from a starting state toward the goal using random samples, making it effective for high-dimensional spaces.

### DRRT (Dynamic RRT)
A dynamic version of RRT that updates its tree structure by pruning invalid branches and re-expanding toward the goal when the environment changes.

## Problem Description

The task addressed in this work is the coordination of a fleet of AMRs within a bounded 2D workspace containing static and dynamic obstacles. The proposed system:
- Initializes multiple robots with defined start and goal positions,
- Plans initial paths using the above algorithms,
- Detects and resolves potential collisions through re-planning,
- Compares the algorithms on metrics such as collision frequency, path length, and computational time.

## Architecture

The system is composed of two primary phases:
1. **Initial Path Planning**: Paths are generated using A*, D*, D* Lite, and DRRT. Linear interpolation is applied to normalize distances between nodes.
2. **Collision Detection & Re-planning**: Robot positions are checked per iteration. If a collision is detected, dynamic re-planning is initiated using the appropriate algorithms.

## Project Structure

Path-Planning-AMR-Warehouse/
- Astar/
  - astar_planner.py
  - astar_simulation.py
  - env.py
  - plotting.py
- Dstar/
  - dstar_planner.py
  - dstar_simulation.py
  - env.py
  - plotting.py
- Dstar Lite/
  - dstar_lite.py
  - dstar_lite_simulation.py
  - env.py
  - plotting.py
- RRT/
  - drrt_simulation.py
  - dynamic_rrt.py
  - env.py
  - grid_rrt.py
  - plotting.py
  - utils.py
- Plots/
  - 1 robot/
  - 2 robots/
  - 4 robots/
  - 6 robots/
- Warehouses/
  - warehouse1.png
  - warehouse2.png
  - warehouse3.png
  - warehouse4.png
- README.md
- Statistics.xlsx

## Features

- Support for dynamic re-planning in changing environments.
- Visual representation of algorithm behavior and path generation.
- Modular design allowing easy testing and extension of algorithms.

## Repository Mention

This project utilizes and adapts code from the following open-source repository:

🔗 **GitHub**: [zhm-real/PathPlanning](https://github.com/zhm-real/PathPlanning/)  
This repository provided a foundational implementation for several of the path planning algorithms integrated in this work.


## Related Papers

* [A*: ](https://ieeexplore.ieee.org/document/4082128) A Formal Basis for the heuristic Determination of Minimum Cost Paths
* [D*: ](http://web.mit.edu/16.412j/www/html/papers/original_dstar_icra94.pdf) Optimal and Efficient Path Planning for Partially-Known Environments
* [D* Lite: ](http://idm-lab.org/bib/abstracts/papers/aaai02b.pdf) D* Lite
* [RRT: ](http://msl.cs.uiuc.edu/~lavalle/papers/Lav98c.pdf) Rapidly-Exploring Random Trees: A New Tool for Path Planning
* [Dynamic-RRT: ](https://www.ri.cmu.edu/pub_files/pub4/ferguson_david_2006_2/ferguson_david_2006_2.pdf) Replanning with RRTs


## License

This project is licensed under the MIT License.
