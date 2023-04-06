# ROS2 Package Template

## Description
This is a template repo with instructions on how to start a ROS2 workspace, package, subscriber, and listener. This ros package and setup tutorial requires Ubuntu 20.04 and ROS2 Foxy Fitzroy. Additional details on ROS2 setup, installation, and operation are in the `ros_help.md` file.

---

## Overview
Fill in this overview section with details on the nodes that are in this package and what they do.

---

## Add Package Template to Workspace

Put the template `my_package` package into your ros workspace `src` directory.

It is a good idea to modify your package before building. 

1. Go to your `package.xml` file and add your package name, package description, maintainer/email, and any required dependencies.
2. Go to `setup.py` and add the maintainer name, maintainer email, package description, and the entry_points for however many nodes you will have in your package.
3. Make sure to modify the overall package name in the `src` directory to whatever you want it set as.

Run the following command to build your new package:
```
colcon build --packages-select my_package
```

If generating from scratch, run a full colcon build
```
colcon build
```

---