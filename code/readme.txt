This directory contains the source code of the reconstruction system.
It is split into five folders:

- launch: This folder contains the launch files for simulation and real execution.

- src_common: This folder contains source code that is common to the simulation mode and the execution on real hardware. Copy the packages within this folder to your catkin workspace.

- src_simulation: This folder contains the packages, which are used to simulate the system. Copy these packages (together with the packages in src_common) to your catkin workspace and compile the code.

- src_real: This folder contains packages, which are used to execute the system on real hardware. Copy these packages (together with the packages in src_common) to your catkin workspace and compile the code.

- twincat_kukalbriiwa_controller: The TwinCAT configuration that was used for the experiments.


Note: The setup including the linear axes was not available at the time of writing. This is why some modifications were necessary to use the system with only 7 instead of 9 degrees of freedom (DoF). The source code in src_common is prepared for 9 DoF. To use the system without linear axes, do the following:

1. Adjust the parameters.yaml file in the acin_gateway/config directory. The values for 7 DoF are commented out.
2. Replace all instances of the word "arm" with "iiwa" in the following files:
   - acin_automation/src/sequence_optimizer.cpp
   - acin_automation/src/view_planner.cpp
   - acin_automation/scripts/script_execution_manager
   - acin_gui/src/object_reconstruction_panel.cpp
3. In acin_moveit/launch/moveit.rviz, replace "Planning Group: arm" with "Planning Group: iiwa"
