### Run the controller with RVIZ visualization

####  1. Preparation:

  As the controller has a lot of dependencies it is recommended you install it using its [superbuild extension](https://github.com/Hugo-L3174/helpup-controller-superbuild).


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
