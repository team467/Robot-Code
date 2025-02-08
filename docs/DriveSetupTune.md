## Setup

This folder includes a predefined AdvantageScope layout with tabs for each setup and tuning step described below. To open it, click `File` > `Import Layout...` in the tab bar of AdvantageScope and select the file `AdvantageScope Swerve Calibration.json` in the swerve project folder.

1. Navigate to `src/main/java/frc/robot/subsystems/drive/DriveConstants.java` in the AdvantageKit project.

2. Update the values of `driveMotorReduction` and `turnMotorReduction` based on the robot's module type and configuration. This information can typically be found on the product page for the swerve module. These values represent reductions and should generally be greater than one.

3. Update the values of `trackWidth` and `wheelBase` based on the distance between the left-right and front-back modules (respectively).

4. Update the value of `wheelRadiusMeters` to the theoretical radius of each wheel. This value can be further refined as described in the "Tuning" section below.

5. Update the value of `maxSpeedMetersPerSec` to the theoretical max speed of the robot. This value can be further refined as described in the "Tuning" section below.

6. For each module, set the value of `...ZeroRotation` to `new Rotation2d(0.0)`.

7. Deploy the project to the robot and connect using AdvantageScope.

8. Check that there are no dashboard alerts or errors in the Driver Station console. If any errors appear, verify that CAN IDs, firmware versions, and configurations of all devices.

9. Manually rotate the turning position of each module such that the position in AdvantageScope (`/Drive/Module.../TurnPosition`) is **increasing**. The module should be rotating **counter-clockwise** as viewed from above the robot. Verify that the units visible in AdvantageScope (radians) match the physical motion of the module. If necessary, change the value of `turnInverted`, `turnEncoderInverted`, or `turnMotorReduction`.

10. Manually rotate each drive wheel and view the position in AdvantageScope (`/Drive/Module.../DrivePositionRad`). Verify that the units visible in AdvantageScope (radians) match the physical motion of the module. If necessary, change the value of `driveMotorReduction`.

11. Manually rotate each module to align it directly forward. **Verify using AdvantageScope that the drive position _increases_ when the wheel rotates such that the robot would be propelled forward.** We recommend pressing a straight object such as aluminum tubing against the pairs of left and right modules to ensure accurate alignment.

12. Record the value of `/Drive/Module.../TurnPosition` for each aligned module. Update the value of `...ZeroRotation` for each module to `new Rotation2d(<insert value>)`.

## Tuning

### Feedforward Characterization

1. Tune turning PID gains as described [here](#driveturn-pid-tuning).

2. Place the robot in an open space.

3. Select the "Drive Simple FF Characterization" auto routine.

4. Enable the robot in autonomous. The robot will slowly accelerate forward, similar to a SysId quasistic test.

5. Disable the robot after at least ~5-10 seconds.

6. Check the console output for the measured `kS` and `kV` values, and copy them to the `driveKs` and `driveKv` constants in `DriveConstants.java`.

### Wheel Radius Characterization

1. Place the robot on carpet. Characterizing on a hard floor may produce errors in the measurement, as the robot's effective wheel radius is affected by carpet compression.

2. Select the "Drive Wheel Radius Characterization" auto routine.

3. Enable the robot in autonomous. The robot will slowly rotate in place.

4. Disable the robot after at least one full rotation.

5. Check the console output for the measured wheel radius, and copy the value to `wheelRadiusMeters` in `DriveConstants.java`.

### Drive/Turn PID Tuning

I recommend using AdvantageScope to plot the measured and setpoint values while tuning. Measured values are published to the `/RealOutputs/SwerveStates/Measured` field and setpoint values are published to the `/RealOutputs/SwerveStates/SetpointsOptimized` field.

### Max Speed Measurement

1. Set `maxSpeedMetersPerSec` in `DriveConstants.java` to the theoretical max speed of the robot based on motor free speed and gearing. This value can typically be found on the product page for your chosen swerve modules.

2. Place the robot in a open space.

3. Plot the measured robot speed in AdvantageScope using the `/RealOutputs/SwerveChassisSpeeds/Measured` field.

4. In teleop, drive forwards at full speed until the robot velocity is no longer increasing.

5. Record the maximum velocity achieved and update the value of `maxSpeedMetersPerSec`.

### Slip Current Measurement

1. Place the robot against the solid wall.

2. Using AdvantageScope, plot the current of a drive motor from the `/Drive/Module.../DriveCurrentAmps` key, and the velocity of the motor from the `/Drive/Module.../DriveVelocityRadPerSec` key.

3. Accelerate forward until the drive velocity increases (the wheel slips). Note the current at this time.

4. Update the value of `driveMotorCurrentLimit` to this value.

### PathPlanner Configuration

- Robot mass, MOI, and wheel coefficient as configured at the bottom of `DriveConstants.java`
- Drive PID constants as configured in `AutoBuilder`.
- Turn PID constants as configured in `AutoBuilder`.