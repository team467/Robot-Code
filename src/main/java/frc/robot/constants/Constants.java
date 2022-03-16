package frc.robot.constants;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import frc.robot.motors.FeedbackConstant;
import frc.robot.motors.GearRatio;
import frc.robot.motors.MotorType;
import frc.robot.motors.RamseteConstant;
import frc.robot.motors.SimpleFeedforwardConstant;
import frc.robot.utilities.IMUAxis;
import frc.robot.utilities.IMUType;

public interface Constants {
  /**
   * @return The name of the robot constant file
   */
  String name();
  /**
   * @return Does the robot have a drivetrain
   */
  boolean hasDrivetrain();
  /**
   * @return Does the drivetrain use 2 motors
   */
  boolean driveDualMotors();
  /**
   * @return What motor type does the drivetrain use
   */
  MotorType driveMotorType();
  /**
   * @return Returns whether the motor is in idle mode and the coasting and braking values.
   *  @see <a href="https://docs.revrobotics.com/sparkmax/operating-modes/idle-mode-brake-coast-mode">More info</a>
   */
  IdleMode driveIdleMode();
  /**
   * @return Does the drivetrain use velocity tuning
   */
  boolean driveUseVelocity();
  /**
   * @return Does the drivetrain use PID tuning
   */
  boolean driveUsePID();
  /**
   * @return Constant for ramsete controller (trajectory tracking) @see
   *     https://docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories/ramsete.html
   */
  RamseteConstant driveRamsete();
  /** TODO: add explanation to what this is */
  SimpleFeedforwardConstant driveDriveFF();
  /** TODO: add explanation to what this is */
  FeedbackConstant driveDriveVelocityPID();
  /** TODO: add explanation to what this is */
  FeedbackConstant driveDrivePositionPID();
  /** TODO: add explanation to what this is */
  SimpleFeedforwardConstant driveTurnFF();
  /** TODO: add explanation to what this is */
  FeedbackConstant driveTurnVelocityPID();
  /** TODO: add explanation to what this is */
  FeedbackConstant driveTurnPositionPID();
  /**
   * @return Returns the diameter of the drive wheels
   */
  double driveWheelDiameter();
  /**
   * @return Constants for gear ratios
   */
  GearRatio driveGearRatio();
  /**
   * @return Constant for DifferentialDriveKinematics (converts between a ChassisSpeeds object and a
   *     DifferentialDriveWheelSpeeds)(@see
   * @see <a href="https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/differential-drive-kinematics.html">More info</a>
   */
  DifferentialDriveKinematics driveKinematics();
  /**
   * @return The maximum drive velocity
   */
  double driveMaxVelocity();
  /**
   * @return The maximum drive acceleration
   */
  double driveMaxAcceleration();
  /**
   * @return The maximum drive velocity for autonomous
   */
  double driveAutoMaxVelocity();
  /**
   * @return The maximum drive acceleration for autonomous
   */
  double driveAutoMaxAcceleration();
  /**
   * @return The maximum drive speed for fast mode
   */
  double driveFastMaxSpeed();
  /**
   * @return The maximum drive speed for normal mode
   */
  double driveNormalMaxSpeed();
  /**
   * @return The maximum drive speed for slow mode
   */
  double driveSlowMaxSpeed();
  /**
   * @return The maximum turn speed for normal mode
   */
  double driveNormalTurnMaxSpeed();
  /**
   * @return The maximum turn speed for slow mode
   */
  double driveSlowTurnMaxSpeed();
  /**
   * @return Returns the left leader motor id
   */
  int driveMotorLeftLeaderId();
  /**
   * @return Is the left leader motor direction inverted
   */
  boolean driveMotorLeftLeaderInverted();
  /**
   * @return Returns the left follower motor id
   */
  int driveMotorLeftFollowerId();
  /**
   * @return Is the left follower motor direction inverted
   */
  boolean driveMotorLeftFollowerInverted();
  /**
   * @return Returns the right leader motor id
   */
  int driveMotorRightLeaderId();
  /**
   * @return Is the right leader motor direction inverted
   */
  boolean driveMotorRightLeaderInverted();
  /**
   * @return Returns the right follower motor id
   */
  int driveMotorRightFollowerId();
  /**
   * @return Is the right follower motor direction inverted
   */
  boolean driveMotorRightFollowerInverted();

  // GYRO
  /**
   * @return Does the robot have a gyro
   */
  boolean hasGyro();
  /**
   * @return The IMU type to use
   */
  IMUType gyroIMUType();
  /**
   * @return The yaw access to use with the gyro
   */
  IMUAxis gyroYawAxis();


  /**
   * @return Does the robot have a 2020 climber
   */
  boolean hasClimber2020();

  int climber2020MotorId();

  boolean climber2020MotorInverted();

  double climber2020UpSpeed();

  double climber2020DownSpeed();


  /**
   * @return Does the robot have a 2022 climber
   */
  boolean hasClimber2022();
  /**
   * @return The motor ID for the right motor
   */
  int climber2022RightMotorId();
  /**
   * @return The motor ID for the left motor
   */
  int climber2022LeftMotorId();
  /**
   * @return Is the left motor inverted
   */
  boolean climber2022LeftMotorInverted();
  /**
   * @return Is the right motor inverted
   */
  boolean climber2022RightMotorInverted();
  /**
   * @return The speed in which the climber goes up
   */
  double climber2022UpSpeed();
  /**
   * @return The speed in which the climber goes down
   */
  double climber2022DownSpeed();
  /**
   * @return The motor ID for the solenoid channel
   */
  int climber2022SolenoidChannel();
  /**
   * @return The limit to how low the left climber goes
   */
  double climber2022LeftLowerLimit();
  /**
   * @return The limit to how low the right climber goes
   */
  double climber2022RightLowerLimit();
  /**
   * @return The limit to how high the left climber goes
   */
  double climber2022LeftUpperLimit();
  /**
   * @return The limit to how high the right climber goes
   */
  double climber2022RightUpperLimit();


  /**
   * @return Does the robot have a 2020 shooter
   */
  boolean hasShooter2020();
  /**
   * @return What motor type does the shooter use
   */
  MotorType shooter2020MotorType();
  /**
   * @return Does the shooter use 2 motors
   */
  boolean shooter2020FlywheelDualMotors();

  int shooter2020FlywheelLeaderMotorId();

  boolean shooter202FlywheelLeaderInverted();

  int shooter2020FlywheelFollowerMotorId();

  boolean shooter202FlywheelFollowerInverted();

  double shooter2020FlywheelDefaultSpeed();

  boolean shooter2020FlywheelUseVelocity();
  /** TODO: add explanation to what this is */
  double shooter2020FlywheelkP();
  /** TODO: add explanation to what this is */
  double shooter2020FlywheelkI();
  /** TODO: add explanation to what this is */
  double shooter2020FlywheelkD();
  /** TODO: add explanation to what this is */
  double shooter2020FlywheelkS();
  /** TODO: add explanation to what this is */
  double shooter2020FlywheelkV();
  /** TODO: add explanation to what this is */
  double shooter2020FlywheelkA();
  /**
   * @return The maximum flywheel 2020 velocity.
   */
  double shooter2020FlywheelkMaxVelocity();
  int shooter2020TriggerMotorId();
  boolean shooter2020TriggerInverted();
  int shooter2020LeftServoId();
  double shooter2020LeftServoMax();
  double shooter2020LeftServoMin();
  int shooter2020RightServoId();
  double shooter2020RightServoMax();
  double shooter2020RightServoMin();


  /**
   * @return Does the robot have a 2022 indexer
   */
  boolean hasIndexer2022();
  /**
   * @return Motor id for indexer motor
   */
  int indexer2022MotorID();
  /**
   * @return Speed of indexer motor when picking up balls
   */
  double indexer2022IdleSpeed();
  /**
   * @return Speed of indexer motor when picking up balls (fast)
   */
  double indexer2022FastSpeed();
  /**
   * @return Speed of indexer motor when spitting out a ball
   */
  double indexer2022OutSpeed();
  /**
   * @return Is the indexer motor inverted
   */
  boolean indexer2022MotorInverted();


  /**
   * @return Does the robot have a 2022 llama neck
   */
  boolean hasLlamaNeck2022();
  /**
   * @return Motor id for llama neck motor
   */
  int llamaNeck2022MotorID();
  /**
   * @return Is the llama neck motor inverted
   */
  boolean llamaNeck2022MotorInverted();
  /**
   * @return Speed of llama neck motor when picking up balls
   */
  double llamaNeck2022IdleSpeed();
  /**
   * @return Speed of llama neck motor when picking up balls (fast)
   */
  double llamaNeck2022InSpeed();
  /**
   * @return Speed of llama neck motor when spitting out a ball
   */
  double llamaNeck2022OutSpeed();
  /**
   * @return Sensor id for upper limit switch
   */
  int llamaNeck2022UpperLimitSwitchChannel();
  /**
   * @return Sensor id for lower limit switch
   */
  int llamaNeck2022LowerLimitSwitchChannel();


  /**
   * @return Does the robot have a 2022 spitter
   */
  boolean hasSpitter2022();
  /**
   * @return The motor ID for the flywheel motor
   */
  int spitter2022MotorId();
  /**
   * @return Is the flywheel motor inverted
   */
  boolean spitter2022MotorInverted();
  /**
   * @return Does the flywheel use velocity tuning
   */
  boolean spitter2022UseVelocity();
  /**
   * @return Does the flywheel use PID tuning
   */
  boolean spitter2022UsePID();
  /** TODO: add explanation to what this is */
  SimpleFeedforwardConstant spitter2022FF();
  /**
   * @return Moment of inertia of the flywheel
   */
  double spitter2022MomentOfInertia();
  /**
   * @return The gear ratio of the spitter subsystem.
   */
  GearRatio spitter2022GearRatio();
  /** TODO: add explanation to what this is */
  FeedbackConstant spitter2022FB();
  /**
   * @return The maximum velocity of the flywheel
   */
  double spitter2022MaxVelocity();
  /**
   * @return The speed to shoot the ball forwards
   */
  double spitter2022ForwardSpeed();
  /**
   * @return The speed to spin the flywheel backwards
   */
  double spitter2022BackwardSpeed();
  /**
   * @return The "m" value in y=mx+b
   */
  double spitter2022DistanceLinearM();
  /**
   * @return The "b" value in y=mx+b
   */
  double spitter2022DistanceLinearB();


  /**
   * @return Does the robot have a hub camera LED
   */
  boolean hasHubCameraLED();
  /**
   * @return The channel ID for the hub camera LED
   */
  int hubCameraLEDChannel();
  /**
   * @return The offset for the Hub Target tracking
   */
  Translation2d hubCameraOffset();

  /**
   * @return The channel ID for the led
   */
  public int ledChannel();

  /**
   * @return Does the robot have a led
   */
  public boolean hasLed2022();

  /**
   * @return The number of leds the robot has
   */
  public int led2022LedCount();
}
