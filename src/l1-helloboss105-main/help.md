# Guidelines and FAQs for Level 1: ROS2 Navigation Assignment

This document is here to provide additional guidance to help you complete the assignment successfully. We’ve also included a Frequently Asked Questions (FAQ) section to address common concerns. If you still have questions, feel free to reach out—details at the end!

---

## Guidelines

- **Start Small**: Approach the assignment step by step. Begin by understanding the simulation environment before diving into the navigation workflows.
- **Use Logs**: ROS2 provides excellent logging to debug issues. Use `ros2 topic echo`, `ros2 run tf2_tools view_frames` and `ros2 action list` to investigate problems.
- **Read the `nav2` documentation**: Specific plugins and their respective configurations are freely available on [Navigation Plugins](https://docs.nav2.org/plugins/index.html) and the 'bringup' section of the open-source [`nav2_bringup`](https://github.com/ros-navigation/navigation2/tree/main/nav2_bringup) repository.
- **Test Components Independently**: Before integrating everything, test map loading, localization, and navigation workflows separately. Follow the KISS principle.
- **Document Your Approach**: Write clear notes about your configurations, challenges, and solutions. This helps us understand your thought process and makes debugging easier.

---

## FAQ

### Q1: I don’t have Ubuntu 22.04. Can I still complete the assignment?
**A**: Yes! Here are some options:
- Use a VM like VirtualBox or VMware to run Ubuntu 22.04.
- Use Docker to create a containerized environment for ROS2 Humble. Check out Docker images for ROS2 Humble [here](https://hub.docker.com/_/ros). However, you will have to setup display permissions for the container to view simulations like Rviz and Gazebo.
  - **For GUI Applications in Docker**:
    - On Ubuntu/Linux: Allow Docker to access the host display using:
      ```bash
      xhost +local:docker
      docker run -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix your_docker_image
      ```
    - On Windows: Install an X Server (e.g., [VcXsrv](https://sourceforge.net/projects/vcxsrv/)) and set the `DISPLAY` variable with your host's IP, e.g.,
      ```bash
      docker run -e DISPLAY=host.docker.internal:0.0 your_docker_image
      ```
    Refer to this [Docker GUI Guide](https://www.baeldung.com/linux/docker-container-gui-applications) for more help.
- Boot into Ubuntu using a multi-boot setup if your hardware supports it. (WARNING! Please be careful while setting up multi-boot, as there is risk of losing data or corrupting your existing OS.)

### Q2: What should I do if the robot times out during navigation?
**A**: Timeouts during navigation are often caused by issues with DDS/RTPS communication. The default DDS implementation (Fast DDS) may cause problems such as:
- Communication failures where sensor data doesn’t reach the necessary callbacks, causing the robot to fail in detecting obstacles.
- Service calls for starting or stopping navigation not functioning as expected.
- Discovery issues in larger systems, leading to unreliable topic connections.

To resolve these issues, we recommend switching to Cyclone DDS, which provides a more stable experience for Nav2. Install Cyclone DDS and set it as your default RMW implementation with the following commands:
```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

For additional details on configuring Cyclone DDS, refer to this guide: [ROS2 Concepts in Practice: DDS/RTPS](https://www.theconstruct.ai/ros2-concepts-in-practice-3-dds-rtps/).

### Q3: My map isn’t loading properly. What’s wrong?
**A**: Double-check the file path in your YAML configuration and ensure the image file is accessible. We recommend using the absolute path to the yaml file. Or, use the `map_server` plugin manually in your terminal to debug the issue independently:
```bash
ros2 run nav2_map_server map_server --ros-args --param yaml_filename:=/path/to/map.yaml
```

### Q4: Localization isn’t working. What should I check?
**A**: Make sure:
- The `scan` topic matches your robot’s sensor topic.
- The AMCL parameters (`min_particles`, `max_particles`, etc.) are correctly tuned.
- The initial pose is set correctly in AMCL params.

### Q5: Can I modify the robot or testbed environment?
**A**: Yes, feel free to make changes to suit your implementation, but ensure your solution works with the provided map and testbed as well.

---

## Need Help?
If you encounter any bugs in the provided code or have questions about the assignment, please don’t hesitate to contact me at **rohit.panikar@ericrobotics.com**.
