package frc.robot.constants;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import frc.robot.motors.FeedbackConstant;
import frc.robot.motors.GearRatio;
import frc.robot.motors.RamseteConstant;
import frc.robot.motors.SimpleFeedforwardConstant;
import frc.robot.motors.MotorType;
import frc.robot.utilities.IMUAxis;
import frc.robot.utilities.IMUType;

public interface Constants {
    public String name();
    public boolean hasDrivetrain();
    public boolean driveDualMotors();
    public MotorType driveMotorType();
    public IdleMode driveIdleMode();
    public boolean driveUseVelocity();
    public boolean driveUsePID();
    public RamseteConstant driveRamsete();
    public SimpleFeedforwardConstant driveDriveFF();
    public FeedbackConstant driveDriveVelocityPID();
    public FeedbackConstant driveDrivePositionPID();
    public SimpleFeedforwardConstant driveTurnFF();
    public FeedbackConstant driveTurnVelocityPID();
    public FeedbackConstant driveTurnPositionPID();
    public double driveWheelDiameter();
    public GearRatio driveGearRatio();
    public DifferentialDriveKinematics driveKinematics();
    public double driveMaxVelocity();
    public double driveMaxAcceleration();
    public double driveAutoMaxVelocity();
    public double driveAutoMaxAcceleration();
    public double driveFastMaxSpeed();
    public double driveNormalMaxSpeed();
    public double driveSlowMaxSpeed();
    public double driveNormalTurnMaxSpeed();
    public double driveSlowTurnMaxSpeed();
    public int driveMotorLeftLeaderId();
    public boolean driveMotorLeftLeaderInverted();
    public int driveMotorLeftFollowerId();
    public boolean driveMotorLeftFollowerInverted();
    public int driveMotorRightLeaderId();
    public boolean driveMotorRightLeaderInverted();
    public int driveMotorRightFollowerId();
    public boolean driveMotorRightFollowerInverted();

    public boolean hasGyro();
    public IMUType gyroIMUType();
    public IMUAxis gyroYawAxis();

    public boolean hasClimber2020();
    public int climber2020MotorId();
    public boolean climber2020MotorInverted();
    public double climber2020UpSpeed();
    public double climber2020DownSpeed();

    public boolean hasShooter2020();
    public MotorType shooter2020MotorType();
    public boolean shooter2020FlywheelDualMotors();
    public int shooter2020FlywheelLeaderMotorId();
    public boolean shooter202FlywheelLeaderInverted();
    public int shooter2020FlywheelFollowerMotorId();
    public boolean shooter202FlywheelFollowerInverted();
    public double shooter2020FlywheelDefaultSpeed();
    public boolean shooter2020FlywheelUseVelocity();
    public double shooter2020FlywheelkP();
    public double shooter2020FlywheelkI();
    public double shooter2020FlywheelkD();
    public double shooter2020FlywheelkS();
    public double shooter2020FlywheelkV();
    public double shooter2020FlywheelkA();
    public double shooter2020FlywheelkMaxVelocity();
    public int shooter2020TriggerMotorId();
    public boolean shooter2020TriggerInverted();
    public int shooter2020LeftServoId();
    public double shooter2020LeftServoMax();
    public double shooter2020LeftServoMin();
    public int shooter2020RightServoId();
    public double shooter2020RightServoMax();
    public double shooter2020RightServoMin();

    public boolean hasIndexer2022();
    public int indexer2022MotorID();
    public double indexer2022IdleSpeed();
    public double indexer2022InSpeed();
    public double indexer2022OutSpeed();
    public boolean indexer2022MotorInverted();


    public boolean hasLlamaNeck2022();
    public int llamaNeck2022MotorID();
    public boolean llamaNeck2022MotorInverted();
    public double llamaNeck2022IdleSpeed();
    public double llamaNeck2022InSpeed();
    public double llamaNeck2022OutSpeed();
    public int llamaNeck2022UpperLimitSwitchChannel();
    public int llamaNeck2022LowerLimitSwitchChannel();

    public boolean hasSpitter2022();
    public int spitter2022MotorId();
    public boolean spitter2022MotorInverted();
    public boolean spitter2022UseVelocity();
    public boolean spitter2022UsePID();
    public SimpleFeedforwardConstant spitter2022FF();
    public double spitter2022MomentOfInertia();
    public GearRatio spitter2022GearRatio();
    public FeedbackConstant spitter2022FB();
    public double spitter2022MaxVelocity();
    public double spitter2022ForwardSpeed();
    public double spitter2022BackwardSpeed();
    public double spitter2022DistanceLinearM();
    public double spitter2022DistanceLinearB();

    public boolean hasHubCameraLED();
    public int hubCameraLEDChannel();
    public Translation2d hubCameraOffset();
}
