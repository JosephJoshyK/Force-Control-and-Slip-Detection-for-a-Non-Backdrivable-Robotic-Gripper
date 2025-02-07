# Force Control and Slip Detection for a Non-Backdrivable Robotic Gripper

## What the Project Does

This project focuses on the development and implementation of a force control and slip detection system for a non-backdrivable robotic gripper. The system aims to enhance the gripper's ability to handle objects with varying levels of grip strength, ensuring that it can securely hold items without causing damage. The project includes the following components:
- Force control algorithms to adjust the gripping force based on real-time feedback.
- Slip detection mechanisms to identify when an object is slipping and adjust the grip accordingly.
- Experimental validation and testing of the system using various objects and scenarios.

### Force Control Demonstration

<p align="center">
  <img src="https://github.com/BijoSebastian/grasp_it_repo/blob/main/Force-Control-and-Slip-Detection-for-a-Non-Backdrivable-Robotic-Gripper-main/Videos/Uncontrolled_grasp.gif" width="300" alt="Uncontrolled Grasp"/>
  <img src="https://github.com/BijoSebastian/grasp_it_repo/blob/main/Force-Control-and-Slip-Detection-for-a-Non-Backdrivable-Robotic-Gripper-main/Videos/Controlled_grasp.gif" width="300" alt="Force Controlled Grasp"/>
</p>
<p align="center">
  <i>Uncontrolled Grasp</i> &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; <i>Force Controlled Grasp</i>
</p>

### Slip Detection Demonstration

<p align="center">
  <img src="https://github.com/JosephJoshyK/Force-Control-and-Slip-Detection-for-a-Non-Backdrivable-Robotic-Gripper/blob/main/Videos/Bottle_slip.gif" width="400" alt="Bottle Slip Detection"/>
</p>
<p align="center">
  <i>Bottle Slip Detection</i>
</p>


## Why the Project is Useful

This project is particularly useful for applications in industrial automation, where precise handling of delicate or variable objects is critical. The ability to control grip force and detect slip in real-time can:
- Reduce damage to sensitive components.
- Improve the efficiency and reliability of automated handling systems.
- Enable the gripper to adapt to a wide range of objects, increasing its versatility.

## How Users Can Get Started with the Project

To get started with this project, follow these steps:

1. **Clone the repository:**
    ```bash
    git clone [https://github.com/JosephJoshyK/Force-Control-and-Slip-Detection-for-a-Non-Backdrivable-Robotic-Gripper.git
    cd Force-Control-and-Slip-Detection-for-a-Non-Backdrivable-Robotic-Gripper
    ```

3. **Setup the Hardware:** Follow the instructions in the `FSR Setup`, `Gripper Setup`, and `Complete Setup` folders to set up the robotic gripper and connect the FSR sensors.

    - **FSR Setup:**
      - Navigate to the `FSR Setup` folder.
      - Follow the detailed setup guide to correctly position and connect the FSR sensors.

    - **Gripper Setup:**
      - Navigate to the `Gripper Setup` folder.
      - Follow the step-by-step instructions to configure the robotic gripper.

    - **Complete Setup:**
      - Navigate to the `Complete Setup` folder.
      - Follow the comprehensive guide to ensure all components are correctly integrated and configured.


4. **Launch the fsr_main codes:**
    ```bash
    roslaunch fsr_main smart_grip.launch
    ```

For more detailed instructions, refer to the `README.md` in each folder.

## Where Users Can Get Help with Your Project

If you encounter any issues or have questions about the project, you can:
- **Open an Issue:** Create a new issue in the GitHub repository for any bugs or feature requests.
- **Discussions:** Participate in discussions on the GitHub Discussions page for this project.
- **Documentation:** Refer to the `README.md` folder for detailed documentation on the system architecture, setup, and usage.

## Acknowledgements

This project is developed to enhance the performance of a non-backdrivable robotic gripper by implementing advanced force control and slip detection mechanisms. Special thanks to the developers of the ROS and Robotiq 3F Gripper control package for their invaluable tools and support.


## Who Maintains and Contributes to the Project

This project is maintained by Joseph Joshy. Contributions are welcome! If you'd like to contribute, please fork the repository and create a pull request with your changes. Ensure your code adheres to the project's coding standards and includes relevant tests.

---

Feel free to reach out if you have any questions or need further assistance. Thank you for your interest in our project!

## Citation

If you find [IKFast](http://openrave.org/docs/0.8.2/openravepy/ikfast/) useful, please cite [OpenRave](http://openrave.org/):

```
@phdthesis{diankov_thesis,
  author = "Rosen Diankov",
  title = "Automated Construction of Robotic Manipulation Programs",
  school = "Carnegie Mellon University, Robotics Institute",
  month = "August",
  year = "2010",
  number= "CMU-RI-TR-10-29",
  url={http://www.programmingvision.com/rosen_diankov_thesis.pdf},
}
```

This module was also a part of [Visual Pushing and Grasping](https://github.com/andyzeng/visual-pushing-grasping). If you find it useful in your work, please consider citing:

```
@inproceedings{zeng2018learning,
  title={Learning Synergies between Pushing and Grasping with Self-supervised Deep Reinforcement Learning},
  author={Zeng, Andy and Song, Shuran and Welker, Stefan and Lee, Johnny and Rodriguez, Alberto and Funkhouser, Thomas},
  booktitle={IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  year={2018}
}
```
