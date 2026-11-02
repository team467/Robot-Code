package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.*;

import frc.robot.Constants;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Scheduler;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.math.linalg.VecBuilder;

class DriveTest {
  private static final FakeModule[] modules = {
    new FakeModule(), new FakeModule(), new FakeModule(), new FakeModule()
  };
  private static final FakeGyro gyro = new FakeGyro();
  private static Drive drive;

  @BeforeAll
  static void setup() throws Exception {
    HAL.initialize();
    // Select simulation without requiring the roboRIO's /home/lvuser/robot file.
    var robotType = Constants.class.getDeclaredField("cachedRobotTypeFromRoborio");
    robotType.setAccessible(true);
    robotType.set(null, Constants.RobotType.ROBOT_SIMBOT);
    drive = new Drive(gyro, modules[0], modules[1], modules[2], modules[3]);
  }

  private void reset() {
    Scheduler.getDefault().cancelAll();
    gyro.connected = true;
    gyro.angle = Rotation2d.ZERO;
    for (var module : modules) {
      module.distance = 0;
      module.angle = Rotation2d.ZERO;
    }
    drive.periodic();
    drive.setPose(Pose2d.ZERO);
  }

  @Test
  void integratesWheelSamplesAndPreservesResetOffset() {
    reset();
    for (var module : modules) module.distance = 1;
    drive.periodic();
    assertEquals(1, drive.getPose().getX(), 1e-9);
    assertEquals(0, drive.getPose().getY(), 1e-9);
    drive.setPose(new Pose2d(4, 5, Rotation2d.fromDegrees(90)));
    for (var module : modules) module.distance = 2;
    drive.periodic();
    assertEquals(4, drive.getPose().getX(), 1e-9);
    assertEquals(6, drive.getPose().getY(), 1e-9);
    assertEquals(Math.PI / 2, drive.getRotation().getRadians(), 1e-9);
  }

  @Test
  void integratesRotationWithoutGyro() {
    reset();
    gyro.connected = false;
    for (int i = 0; i < modules.length; i++) {
      var location = DriveConstants.moduleTranslations[i];
      modules[i].angle = new Rotation2d(-location.getY(), location.getX());
      modules[i].distance = location.getNorm() * 0.2;
    }
    drive.periodic();
    assertEquals(0.2, drive.getRotation().getRadians(), 1e-9);
    assertEquals(0, drive.getPose().getTranslation().getNorm(), 1e-9);
  }

  @Test
  void retainsTimestampedVisionCorrection() {
    reset();
    for (var module : modules) module.distance = 1;
    drive.periodic();
    drive.addVisionMeasurement(
        new Pose2d(1.2, 0, Rotation2d.ZERO), modules[0].timestamp, VecBuilder.fill(0, 0, 0));
    assertEquals(1.2, drive.getPose().getX(), 1e-9);
    for (var module : modules) module.distance = 2;
    drive.periodic();
    assertEquals(2.2, drive.getPose().getX(), 1e-9);
  }

  @Test
  void optimizesAndCosineScalesAgainstMeasuredAngle() {
    FakeModule io = new FakeModule();
    Module module = new Module(io, 4);
    module.periodic();
    var applied = module.runSetpoint(new SwerveModuleVelocity(2, Rotation2d.fromDegrees(60)));
    assertEquals(1, applied.velocity, 1e-9);
    assertEquals(1 / DriveConstants.wheelRadiusMeters, io.velocity, 1e-9);
    applied = module.runSetpoint(new SwerveModuleVelocity(2, Rotation2d.fromDegrees(180)));
    assertEquals(-2, applied.velocity, 1e-9);
    assertEquals(0, applied.angle.getRadians(), 1e-9);
  }

  @Test
  void lockCanBeEnteredRepeatedlyAndCancellationStopsDrive() {
    reset();
    var scheduler = Scheduler.getDefault();
    var lock = new AtomicBoolean();
    var ticks = new AtomicInteger();
    var control =
        drive
            .runRepeatedly(
                () -> {
                  ticks.incrementAndGet();
                  drive.runVelocity(new org.wpilib.math.kinematics.ChassisVelocities(1, 0, 0));
                })
            .named("Test driver");
    var command = drive.defaultDriveCommand(control, lock::get);
    assertTrue(scheduler.schedule(command).successful());
    for (int repeat = 0; repeat < 3; repeat++) {
      for (int i = 0; i < 5; i++) scheduler.run();
      int beforeLock = ticks.get();
      assertTrue(beforeLock > 0);
      lock.set(true);
      for (int i = 0; i < 5; i++) scheduler.run();
      int lockedTicks = ticks.get();
      for (int i = 0; i < 5; i++) scheduler.run();
      assertEquals(lockedTicks, ticks.get());
      for (var module : modules) assertEquals(0, module.velocity, 1e-9);
      lock.set(false);
      for (int i = 0; i < 5; i++) scheduler.run();
      assertTrue(ticks.get() > lockedTicks);
    }
    scheduler.cancel(command);
    for (var module : modules) assertEquals(0, module.velocity, 1e-9);
  }

  @Test
  void fieldRelativeJoystickRequestUsesInverseHeading() {
    reset();
    for (var module : modules) module.angle = Rotation2d.fromDegrees(-90);
    drive.periodic();
    drive.setPose(new Pose2d(0, 0, Rotation2d.fromDegrees(90)));
    var scheduler = Scheduler.getDefault();
    var command =
        frc.robot.commands.drive.DriveCommands.joystickDrive(drive, () -> 1, () -> 0, () -> 0);
    assertTrue(scheduler.schedule(command).successful());
    for (int i = 0; i < 3; i++) scheduler.run();
    for (var module : modules) {
      assertEquals(
          DriveConstants.maxSpeedMetersPerSec / DriveConstants.wheelRadiusMeters,
          module.velocity,
          1e-8);
      assertEquals(-Math.PI / 2, module.setpointAngle.getRadians(), 1e-9);
    }
    scheduler.cancel(command);
  }

  @Test
  void handlesIncompleteGyroSamplesWithWheelFallback() {
    reset();
    gyro.missingSamples = true;
    for (var module : modules) module.distance = 0.5;
    try {
      assertDoesNotThrow(drive::periodic);
      assertEquals(0.5, drive.getPose().getX(), 1e-9);
    } finally {
      gyro.missingSamples = false;
    }
  }

  private static class FakeGyro implements GyroIO {
    boolean connected = true;
    boolean missingSamples;
    Rotation2d angle = Rotation2d.ZERO;

    @Override
    public void updateInputs(GyroIOInputs inputs) {
      inputs.connected = connected;
      inputs.yawPosition = angle;
      inputs.odometryYawPositions = missingSamples ? new Rotation2d[0] : new Rotation2d[] {angle};
    }
  }

  private static class FakeModule implements ModuleIO {
    double distance;
    double velocity;
    double timestamp;
    Rotation2d angle = Rotation2d.ZERO;
    Rotation2d setpointAngle = Rotation2d.ZERO;

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
      timestamp += 0.02;
      inputs.driveConnected = true;
      inputs.turnConnected = true;
      inputs.drivePositionRad = distance / DriveConstants.wheelRadiusMeters;
      inputs.turnPosition = angle;
      inputs.odometryTimestamps = new double[] {timestamp};
      inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionRad};
      inputs.odometryTurnPositions = new Rotation2d[] {angle};
    }

    @Override
    public void setTurnPosition(Rotation2d angle) {
      setpointAngle = angle;
    }

    @Override
    public void setDriveVelocity(double value) {
      velocity = value;
    }

    @Override
    public void setDriveOpenLoop(double output) {
      if (output == 0) velocity = 0;
    }
  }
}
