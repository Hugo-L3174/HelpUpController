### Run the controller with RVIZ simulation

####  1. Preparation:

  * Install the dependencies:
       1. [mc_rtc](https://jrl-umi3218.github.io/mc_rtc/tutorials/introduction/installation-guide.html)
       2. [stabiliplus](https://gite.lirmm.fr/mc-controllers/stabiliplus) The balance polytope computation library
       3. eigen-quadprog and sch-core
       4. [gram_savitzky_golay](https://github.com/arntanguy/gram_savitzky_golay) a filtering and derivation library. The branch used is topic/sva.

  * Install the robot models and their module in mc_rtc:
       1. [human_description](https://gite.lirmm.fr/hlefevre/human_description) is the human model to download in your catkin data workspace.
       2. [mc_human](https://gite.lirmm.fr/hlefevre/mc_human) is the interface between `mc_rtc` and the human model.
       1. [hrp4_description](https://gite.lirmm.fr/hlefevre/hrp4) is the robot model to download in your catkin data workspace.
       4. [mc_hrp4](https://gite.lirmm.fr/mc-hrp4/mc-hrp4) is the interface between `mc_rtc` and the HRP4 model.

  * Install the required plugins:
       1. [mc_xsens_plugin]() The plugin used to get the xsens data from the motion capture and create the datastore calls necessary to get the data.
       2. [mc_force_shoe_plugin](https://github.com/Hugo-L3174/mc_force_shoe_plugin) The plugin used to get the 6d force vectors from the xsens force shoes prototypes.

  * Set the main robot and the controller in your `mc_rtc.yaml` configuration file:
```yaml
  MainRobot: HRP4NoHand
  Enabled: HelpUpController
```



####  2. Run the controller:

  * Start `roscore`:
```sh
roscore
```

  * Start `fsm_withHuman_display.rviz`:
```sh
cd launch/
rosrun rviz rviz -d fsm_withHuman_display.rviz
```

  * Run `mc_rtc_ticker`:
```sh
rosrun mc_rtc_ticker mc_rtc_ticker
```

####  Notes

 - The FSM is configured with the ResetPostures parameter, which updates the background posture task to the current pose at transitions. This allows smoother movements at the holding phase, and avoids singularities on the left elbow.

 - The body configuration of the robot to take the position on the human is "artificially" influenced by a posture task on the left elbow (shoulder + wrist   joints) to "preplan" the local minima the robot can get stuck in with its limited degrees of freedom.

 - The DCM objective of the robot enforced by the stabilizer is scaled to the x axis of the human laterally while the robot takes position, and then back to the center of the polytope: this is to minimize the problem induced by the position error from the force control of the feet during the standup motion, but still take the position correctly in the beginning (otherwise extreme for the left hand because of the robot limitations)
