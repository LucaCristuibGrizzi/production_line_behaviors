# Production Line Behaviors Package

This package contains all production_line-specific states and behaviors. There are two behaviors:

1. **[sequential_production_line](production_line_flexbe_behaviors/src/production_line_flexbe_behaviors/sequential_production_line_sm.py)**: The operations of the production line are computed in a sequential way, and only one product is assembled.

2. **[production_line](production_line_flexbe_behaviors/src/production_line_flexbe_behaviors/production_line_sm.py)**: The operation of the production line is computed in a parallel way, and three products are assembled. Furthermore, this behavior manages the errors that can occur in the production process.

For more information about the production line, look at the [production_line_device](https://github.com/LucaCristuibGrizzi/production_line_device "production_line_device") package.

At this [page](https://doi.org/10.5281/zenodo.10210928 "page for videos of the producition line"), you can find videos showcasing the behaviors of the production line and the error management.

This package is intended to be used with another two packages:
- [production_line_device](https://github.com/LucaCristuibGrizzi/production_line_device "production_line_device")
- [rotating_table](https://github.com/LucaCristuibGrizzi/rotating_table "rotating_table")

## Installation

First of all, it is mandatory to have installed FlexBE. For doing so, you can find the instructions in two places:

1. [**Official Website**](http://philserver.bplaced.net/fbe/download.php "FlexBE site").

2. [**GitHub repository**](https://github.com/FlexBE/flexbe_behavior_engine/tree/noetic "GitHub FlexBE"). Remember to select the correct branch that corresponds to the ROS release that you are using.

It is highly recommended to follow the ROS tutorial of FlexBE ([FlexBE tutorial](http://wiki.ros.org/flexbe/Tutorials "FlexBE tutorial")).

Clone the following repository into your ROS workspace:

```bash
git clone https://github.com/LucaCristuibGrizzi/production_line_behaviors
```

Build you workspace:

```bash
catkin_make # or catkin build
```

It is possible that FlexBE may not immediately detect the new states and behaviors. To address this issue, you can open the *Configuration* tab of the graphical interface of the FlexBE app and click on the *Force Discover* button.

## Libraries Version

- **FlexBE App**: 2.3.0
- **FlexBE Behavior Engine**: 1.4.0