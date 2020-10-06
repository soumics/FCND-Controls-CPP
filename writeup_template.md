# The C++ Project Writeup #

I am sorry that I have not found instructions to submit a writeup before the submission in rubric! Honest mistake! The writeup is not like in the previous project! I mean there was no sample template like in the previous project! So I have copied the README.md and discussed each scenario. Sorry for that! Please go through the ###Comments part only after each paragraph copied from the README.md! Thanks!


###Comments after second review######################################################################################

changes made in RollPitchControl

  target_R13 = -CONSTRAIN(accelCmd[0] / (collThrustCmd / mass), -maxTiltAngle, maxTiltAngle);
  target_R23 = -CONSTRAIN(accelCmd[1] / (collThrustCmd / mass), -maxTiltAngle, maxTiltAngle);
  
new controller parameters:

# Position control gains
kpPosXY = 25 #15 [increased position gain from 22 to 25]
kpPosZ = 30 #26
KiPosZ = 42

# Velocity control gains
kpVelXY = 10 #7
kpVelZ = 9 #8

# Angle control gains
kpBank = 10
kpYaw = 2

# Angle rate gains
kpPQR = 87, 87, 7


And everything has worked! It passes the Scenario 5 and all! 

##########################################################################################################################

### Testing it Out ###

When you run the simulator, you'll notice your quad is falling straight down.  This is due to the fact that the thrusts are simply being set to:

```
QuadControlParams.Mass * 9.81 / 4
```

Therefore, if the mass doesn't match the actual mass of the quad, it'll fall down.  Take a moment to tune the `Mass` parameter in `QuadControlParams.txt` to make the vehicle more or less stay in the same spot.

Note: if you want to come back to this later, this scenario is "1_Intro".

With the proper mass, your simulation should look a little like this:

<p align="center">
<img src="animations/scenario1.gif" width="500"/>
</p>


###Comments############################################################################

Honestly, I have consulted the code from the following link of the Knowledge section!

https://knowledge.udacity.com/questions/311735

#########################################################################################

## The Tasks ##

For this project, you will be building a controller in C++.  You will be implementing and tuning this controller in several steps.

You may find it helpful to consult the [Python controller code](https://github.com/udacity/FCND-Controls/blob/solution/controller.py) as a reference when you build out this controller in C++.

#### Notes on Parameter Tuning
1. **Comparison to Python**: Note that the vehicle you'll be controlling in this portion of the project has different parameters than the vehicle that's controlled by the Python code linked to above. **The tuning parameters that work for the Python controller will not work for this controller**

2. **Parameter Ranges**: You can find the vehicle's control parameters in a file called `QuadControlParams.txt`. The default values for these parameters are all too small by a factor of somewhere between about 2X and 4X. So if a parameter has a starting value of 12, it will likely have a value somewhere between 24 and 48 once it's properly tuned.

3. **Parameter Ratios**: In this [one-page document](https://www.overleaf.com/read/bgrkghpggnyc#/61023787/) you can find a derivation of the ratio of velocity proportional gain to position proportional gain for a critically damped double integrator system. The ratio of `kpV / kpP` should be 4.

###Comments###########################################################################################

From the above link, the code follows

    float B, C, D;
    B = sqrt(2) * momentCmd.x / L;
    C = sqrt(2) * momentCmd.y / L;
    D = -momentCmd.z / kappa;
    cmd.desiredThrustsN[0] = CONSTRAIN((collThrustCmd + B + C + D) / 4.f, minMotorThrust, maxMotorThrust); // front left
    cmd.desiredThrustsN[1] = CONSTRAIN((collThrustCmd - B + C - D) / 4.f, minMotorThrust, maxMotorThrust); // front right
    cmd.desiredThrustsN[2] = CONSTRAIN((collThrustCmd + B - C - D) / 4.f, minMotorThrust, maxMotorThrust); // rear left
    cmd.desiredThrustsN[3] = CONSTRAIN((collThrustCmd - B - C + D) / 4.f, minMotorThrust, maxMotorThrust); // rear right
    
As you have already pointed out in the review that the motor commands do not need to be constrained, and I also have the doubt, because we are already constraining the commands in the controller! In the new version I have removed the Contrain(...) part!

   float l = L * M_SQRT1_2;
  
  //collThrustCmd = mass * 9.81f;
        
  float c_bar = collThrustCmd;
  float p_bar = momentCmd.x / l;
  float q_bar = momentCmd.y / l;
  float r_bar = -momentCmd.z / kappa; // thrust up, momentCmd is down
        
  
  // apparently the system accepts thrust as sqrt(F/k_f) according to F = k_f * omega^2, so no need to calc actual F per rotor
  cmd.desiredThrustsN[0] = (c_bar + p_bar + q_bar + r_bar) / 4.f;
  cmd.desiredThrustsN[1] = (c_bar - p_bar + q_bar - r_bar) / 4.f;
  cmd.desiredThrustsN[2] = (c_bar + p_bar - q_bar - r_bar) / 4.f;
  cmd.desiredThrustsN[3] = (c_bar - p_bar - q_bar - r_bar) / 4.f;

#############################################################################################################################################

### Body rate and roll/pitch control (scenario 2) ###

First, you will implement the body rate and roll / pitch control.  For the simulation, you will use `Scenario 2`.  In this scenario, you will see a quad above the origin.  It is created with a small initial rotation speed about its roll axis.  Your controller will need to stabilize the rotational motion and bring the vehicle back to level attitude.

To accomplish this, you will:

1. Implement body rate control

 - implement the code in the function `GenerateMotorCommands()`
 - implement the code in the function `BodyRateControl()`
 - Tune `kpPQR` in `QuadControlParams.txt` to get the vehicle to stop spinning quickly but not overshoot

If successful, you should see the rotation of the vehicle about roll (omega.x) get controlled to 0 while other rates remain zero.  Note that the vehicle will keep flying off quite quickly, since the angle is not yet being controlled back to 0.  Also note that some overshoot will happen due to motor dynamics!.

If you come back to this step after the next step, you can try tuning just the body rate omega (without the outside angle controller) by setting `QuadControlParams.kpBank = 0`.

2. Implement roll / pitch control
We won't be worrying about yaw just yet.

 - implement the code in the function `RollPitchControl()`
 - Tune `kpBank` in `QuadControlParams.txt` to minimize settling time but avoid too much overshoot

If successful you should now see the quad level itself (as shown below), though it’ll still be flying away slowly since we’re not controlling velocity/position!  You should also see the vehicle angle (Roll) get controlled to 0.

<p align="center">
<img src="animations/scenario2.gif" width="500"/>
</p>

###Comments############################################################################################################################

The two implementations are same as in the Python exercise and in the tutorial!

########################################################################################################################################

### Position/velocity and yaw angle control (scenario 3) ###

Next, you will implement the position, altitude and yaw control for your quad.  For the simulation, you will use `Scenario 3`.  This will create 2 identical quads, one offset from its target point (but initialized with yaw = 0) and second offset from target point but yaw = 45 degrees.

 - implement the code in the function `LateralPositionControl()`
 - implement the code in the function `AltitudeControl()`
 - tune parameters `kpPosZ` and `kpPosZ`
 - tune parameters `kpVelXY` and `kpVelZ`

If successful, the quads should be going to their destination points and tracking error should be going down (as shown below). However, one quad remains rotated in yaw.

 - implement the code in the function `YawControl()`
 - tune parameters `kpYaw` and the 3rd (z) component of `kpPQR`

Tune position control for settling time. Don’t try to tune yaw control too tightly, as yaw control requires a lot of control authority from a quadcopter and can really affect other degrees of freedom.  This is why you often see quadcopters with tilted motors, better yaw authority!

<p align="center">
<img src="animations/scenario3.gif" width="500"/>
</p>

**Hint:**  For a second order system, such as the one for this quadcopter, the velocity gain (`kpVelXY` and `kpVelZ`) should be at least ~3-4 times greater than the respective position gain (`kpPosXY` and `kpPosZ`).

###Comments###############################################################################################################################

From the link that I have consulted, the code for altitudeControl is as follows

  float h_dot_cmd, accel_cmd;
  h_dot_cmd = CONSTRAIN(kpPosZ * (posZCmd - posZ) + velZCmd, -maxAscentRate, maxDescentRate);
  integratedAltitudeError += (posZCmd - posZ) * dt;
  accel_cmd = accelZCmd + kpVelZ * (h_dot_cmd - velZ) + kpPosZ * (posZCmd - posZ) + KiPosZ * integratedAltitudeError;
  thrust = CONSTRAIN(mass * (9.81f - accel_cmd) / R(2, 2), 4.f * minMotorThrust, 4 * maxMotorThrust);
  
  I literally had no clue why h_dot_cmd is necessary! our system is "x_dot_dot=u", and "u" should be designed like "u=kp*e_p+kv*e_v+ki*area under the curve of e_p". It is nothing like x_dot=u_1, and x_dot_dot=u_2=f(u_1,...). So I have deleted the h_dot_cmd line and instead written as follows
  
  float accel_cmd;
  integratedAltitudeError += (posZCmd - posZ) * dt;
  accel_cmd = accelZCmd + kpVelZ * (velZCmd - velZ) + kpPosZ * (posZCmd - posZ) + KiPosZ * integratedAltitudeError;
  
  
  With the same argument, the lateralpositioncontrol function was also written like
  
  V3F velXYCmd;
  velXYCmd = kpPosXY * (posCmd - pos) + velCmd;
  velXYCmd.x = CONSTRAIN(velCmd.x, -maxSpeedXY, maxSpeedXY);
  velXYCmd.y = CONSTRAIN(velCmd.y, -maxSpeedXY, maxSpeedXY);
  accelCmd = accelCmdFF + kpVelXY * (velXYCmd - vel) + kpPosXY * (posCmd - pos);
  accelCmd.x = CONSTRAIN(accelCmd.x, -maxAccelXY, maxAccelXY);
  accelCmd.y = CONSTRAIN(accelCmd.y, -maxAccelXY, maxAccelXY);
  accelCmd.z = 0.f;
  
  which I have felt, should be changed to the following
  
  accelCmd = accelCmdFF + kpVelXY * (velCmd - vel) + kpPosXY * (posCmd - pos);
  
  accelCmd.x = CONSTRAIN(accelCmd.x, -maxAccelXY, maxAccelXY);
  accelCmd.y = CONSTRAIN(accelCmd.y, -maxAccelXY, maxAccelXY);
  accelCmd.z = 0.f;
  
  I have removed the velocity part!

### Non-idealities and robustness (scenario 4) ###

In this part, we will explore some of the non-idealities and robustness of a controller.  For this simulation, we will use `Scenario 4`.  This is a configuration with 3 quads that are all are trying to move one meter forward.  However, this time, these quads are all a bit different:
 - The green quad has its center of mass shifted back
 - The orange vehicle is an ideal quad
 - The red vehicle is heavier than usual

1. Run your controller & parameter set from Step 3.  Do all the quads seem to be moving OK?  If not, try to tweak the controller parameters to work for all 3 (tip: relax the controller).

2. Edit `AltitudeControl()` to add basic integral control to help with the different-mass vehicle.

3. Tune the integral control, and other control parameters until all the quads successfully move properly.  Your drones' motion should look like this:

<p align="center">
<img src="animations/scenario4.gif" width="500"/>
</p>

###Comments#################################################################################################################################

The addition of the integral control, I think, might make the controlled system, a bit, "jerky" (may be due to the effect of mass, boo!, not sure!), as we find "jerks"/switching on the sliding surface of the sliding mode controller!

############################################################################################################################################


### Tracking trajectories ###

Now that we have all the working parts of a controller, you will put it all together and test it's performance once again on a trajectory.  For this simulation, you will use `Scenario 5`.  This scenario has two quadcopters:
 - the orange one is following `traj/FigureEight.txt`
 - the other one is following `traj/FigureEightFF.txt` - for now this is the same trajectory.  For those interested in seeing how you might be able to improve the performance of your drone by adjusting how the trajectory is defined, check out **Extra Challenge 1** below!

How well is your drone able to follow the trajectory?  It is able to hold to the path fairly well?


### Extra Challenge 1 (Optional) ###

You will notice that initially these two trajectories are the same. Let's work on improving some performance of the trajectory itself.

1. Inspect the python script `traj/MakePeriodicTrajectory.py`.  Can you figure out a way to generate a trajectory that has velocity (not just position) information?

2. Generate a new `FigureEightFF.txt` that has velocity terms
Did the velocity-specified trajectory make a difference? Why?

With the two different trajectories, your drones' motions should look like this:

<p align="center">
<img src="animations/scenario5.gif" width="500"/>
</p>

###Comments#################################################################################################################################

C'mmon, amico mio/ amica mia [my friend]! This step has literally left me frustrated!

With my set of gains, 

# Position control gains
kpPosXY = 15
kpPosZ = 15
KiPosZ = 8

# Velocity control gains
kpVelXY = 5
kpVelZ = 5

# Angle control gains
kpBank = 11
kpYaw = 2

# Angle rate gains
kpPQR = 50, 50, 5



I confess that, it has not passed the rubric evaluation criteria! But when it comes to a lot of quads (number>3), you can see all the quads are moving without a fall/ crush!

But with your suggested gains

# Position control gains
kpPosXY = 22
kpPosZ = 30 [I can not go beyond 26! it crushes!]
KiPosZ = 42

# Velocity control gains
kpVelXY = 10
kpVelZ = 9

# Angle control gains
kpBank = 10
kpYaw = 2

# Angle rate gains
kpPQR = 87, 87, 7

I can not increase kpPosZ over 26! It crashes! Also quads>3 is not working! [No offense, please!, because the rest of the gains are yours]! But I still can not pass the 'trajectory'-following criteria, even if the curve in your review, is the same for quad2! Please help!

### Extra Challenge 2 (Optional) ###

For flying a trajectory, is there a way to provide even more information for even better tracking?

How about trying to fly this trajectory as quickly as possible (but within following threshold)!



## Authors ##

Thanks to Fotokite for the initial development of the project code and simulator.
