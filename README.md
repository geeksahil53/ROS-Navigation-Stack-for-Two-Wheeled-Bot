# ROS Navigation Stack for Two-Wheeled Bot

This repository contains a ROS (Robot Operating System) navigation stack implementation for a two-wheeled robot. The navigation stack enables the robot to autonomously navigate its environment using Simultaneous Localization and Mapping (SLAM) techniques and path planning algorithms.

## Contents

- **motion_plan folder:**
  - **pid_final.py:** This Python script implements a PID controller for the bot to center itself between two walls if needed. It assists in maintaining a desired position within the environment.
  - **gmap.launch:** Launch file for SLAM (Simultaneous Localization and Mapping) using the Gmapping package. This launches the necessary nodes to perform mapping of the environment.

- **nav_bot_stack folder:**
  - Contains the main navigation stack for the two-wheeled bot.
  - Utilizes the AMCL (Adaptive Monte Carlo Localization) package for localization.
  - Uses the move_base package for path planning and navigation.
  - Config folder includes various parameter files for configuring costmaps, local costmaps, and other parameters required for navigation.

## Installation

1. Clone this repository into your ROS workspace:

    ```
    git clone https://github.com/geeksahil53/your_repository.git
    ```

2. Build your ROS workspace:

    ```
    cd your_workspace
    catkin_make
    ```

## Usage

1. Launch the SLAM node to start mapping the environment:

    ```
    roslaunch motion_plan gmap.launch
    ```

2. Launch the navigation stack for autonomous navigation. Ensure that the map generated by Gmapping is available:

    ```
    roslaunch nav_bot_stack nav.launch map_file:=/path/to/your/map.yaml
    ```

3. Use RViz or other visualization tools to monitor the robot's navigation process and view the generated map.

## Acknowledgments

- This project builds upon the ROS navigation stack and various ROS packages.
- Refer to ROS documentation for more details
- Special thanks to the ROS community for their contributions and support.

