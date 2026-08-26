package frc.robot.sim;

import static frc.robot.subsystems.shooter.ShooterConstants.kShooterOffsetFromRobotCenter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.FieldConstants.Hub;
import frc.robot.FieldConstants.LinesVertical;
import frc.robot.RobotState;
import frc.robot.RobotState.IntakePosition;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.extend.IntakeExtend;
import frc.robot.subsystems.intake.rollers.IntakeRollers;
import frc.robot.subsystems.magicCarpet.MagicCarpet;
import frc.robot.subsystems.shooter.Shooter;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Iterator;
import java.util.List;
import java.util.Random;
import org.littletonrobotics.junction.Logger;

/**
 * Simulates ball intake from the field, storage inside the robot, and shooting physics into the
 * 2026 Hub.
 */
public class BallSimulator {
  private static BallSimulator instance;

  public static final int MAX_CAPACITY = 8;
  public static final double GRAVITY = 9.81; // m/s^2
  public static final double LAUNCH_ANGLE_RAD = Math.toRadians(65.0); // 65 deg hood angle
  public static final double SHOOTER_HEIGHT_M = 0.50; // Initial release height off the carpet
  public static final double HUB_HEIGHT_M = Hub.height; // 1.8288 m (72 inches)
  public static final double HUB_TARGET_RADIUS_M = Hub.width / 2.0; // ~0.597 m (23.5 in radius)
  public static final double SHOT_INTERVAL_SECONDS = 0.28; // Min time between successive balls fed
  public static final double RPM_TO_EXIT_VELOCITY = 0.00525; // Calibrated launch velocity per RPM

  // Intake probabilities per 20ms simulation loop when driving with intake on
  private static final double NEUTRAL_ZONE_INTAKE_PROB = 0.055; // ~2.75 balls/sec
  private static final double ALLIANCE_ZONE_INTAKE_PROB = 0.015; // ~0.75 balls/sec

  private int ballsInRobot = 0;
  private int ballsScoredInHub = 0;
  private int totalShotsAttempted = 0;
  private int totalShotsMissed = 0;

  private double lastShotTimestamp = 0.0;
  private boolean lastShotSuccess = false;
  private double lastShotDistance = 0.0;

  private final Random random = new Random();
  private final List<FlyingBall> activeFlyingBalls =
      Collections.synchronizedList(new ArrayList<>());

  public static synchronized BallSimulator getInstance() {
    if (instance == null) {
      instance = new BallSimulator();
    }
    return instance;
  }

  private BallSimulator() {}

  /**
   * Main periodic update method called during simulation to evaluate intake and shooting physics.
   */
  public void update(
      Drive drive,
      Shooter shooter,
      Indexer indexer,
      MagicCarpet magicCarpet,
      IntakeRollers intakeRollers,
      IntakeExtend intakeExtend) {
    if (drive == null) return;

    double now = Timer.getFPGATimestamp();
    Pose2d robotPose = drive.getPose();
    double chassisSpeed =
        Math.hypot(
            drive.getChassisSpeeds().vxMetersPerSecond, drive.getChassisSpeeds().vyMetersPerSecond);

    // 1. Process Intake Logic
    boolean isIntakeRunning = RobotState.getInstance().intaking;
    boolean isIntakeDeployed = RobotState.getInstance().intakePosition == IntakePosition.DEPLOYED;
    boolean isMoving = chassisSpeed > 0.08;

    if (isIntakeRunning && isIntakeDeployed && isMoving && ballsInRobot < MAX_CAPACITY) {
      double intakeProbability = getIntakeProbability(robotPose.getX());
      if (random.nextDouble() < intakeProbability) {
        ballsInRobot++;
        Logger.recordOutput("BallSim/IntakeEvent", true);
      }
    }

    // 2. Process Shooting Logic
    boolean isShooterRunning =
        RobotState.getInstance().shooterAtSpeed || shooter.getSetpoint() > 10.0;
    boolean isIndexerRunning =
        RobotState.getInstance().indexerRunning || indexer.getVoltage() > 1.0;
    boolean isCarpetRunning = magicCarpet != null; // Magic carpet runs with indexer

    if (ballsInRobot > 0
        && isShooterRunning
        && isIndexerRunning
        && (now - lastShotTimestamp) >= SHOT_INTERVAL_SECONDS) {
      fireBall(drive, shooter, robotPose, now);
    }

    // 3. Update Flying Projectiles
    updateFlyingBalls(now);

    // 4. Log simulation telemetry
    Logger.recordOutput("BallSim/BallsInRobot", ballsInRobot);
    Logger.recordOutput("BallSim/BallsScoredInHub", ballsScoredInHub);
    Logger.recordOutput("BallSim/TotalShotsAttempted", totalShotsAttempted);
    Logger.recordOutput("BallSim/LastShotSuccess", lastShotSuccess);
  }

  /** Determines intake chance based on field X position (Alliance vs Neutral zones). */
  private double getIntakeProbability(double fieldX) {
    // Neutral zone is located between neutralZoneNear and neutralZoneFar
    if (fieldX >= LinesVertical.neutralZoneNear && fieldX <= LinesVertical.neutralZoneFar) {
      return NEUTRAL_ZONE_INTAKE_PROB;
    } else {
      return ALLIANCE_ZONE_INTAKE_PROB;
    }
  }

  /** Fires a ball, performs projectile trajectory math, and checks if it enters the Hub. */
  private void fireBall(Drive drive, Shooter shooter, Pose2d robotPose, double now) {
    ballsInRobot--;
    totalShotsAttempted++;
    lastShotTimestamp = now;

    Translation2d shooterPos = getShooterPosition(robotPose);
    Translation2d hubTarget = getTargetHubCenter();
    double distanceToHub = shooterPos.getDistance(hubTarget);
    lastShotDistance = distanceToHub;

    // Ball launch speed calculated from shooter RPM / setpoint
    double currentRPM =
        shooter.getSetpoint() > 0 ? (shooter.getSetpoint() * 60.0 / (2 * Math.PI)) : 1000.0;
    double exitVelocity = currentRPM * RPM_TO_EXIT_VELOCITY;

    double v0h = exitVelocity * Math.cos(LAUNCH_ANGLE_RAD);
    double v0z = exitVelocity * Math.sin(LAUNCH_ANGLE_RAD);

    // Add robot velocity vector
    double heading = robotPose.getRotation().getRadians();
    double robotVx = drive.getChassisSpeeds().vxMetersPerSecond;
    double robotVy = drive.getChassisSpeeds().vyMetersPerSecond;

    double ballVx = v0h * Math.cos(heading) + robotVx;
    double ballVy = v0h * Math.sin(heading) + robotVy;
    double effVh = Math.hypot(ballVx, ballVy);

    // Calculate time to reach Hub height on descending trajectory
    double deltaZ = HUB_HEIGHT_M - SHOOTER_HEIGHT_M;
    double discriminant = v0z * v0z - 2 * GRAVITY * deltaZ;

    boolean hit = false;
    double timeToHub = 1.0;
    Translation2d landingPos = shooterPos;

    if (discriminant >= 0) {
      // Descending crossing time (falling into top of hub)
      timeToHub = (v0z + Math.sqrt(discriminant)) / GRAVITY;
      double landX = shooterPos.getX() + ballVx * timeToHub;
      double landY = shooterPos.getY() + ballVy * timeToHub;
      landingPos = new Translation2d(landX, landY);

      double landingError = landingPos.getDistance(hubTarget);
      if (landingError <= HUB_TARGET_RADIUS_M) {
        hit = true;
      }
    }

    lastShotSuccess = hit;
    if (hit) {
      ballsScoredInHub++;
    } else {
      totalShotsMissed++;
    }

    // Register flying projectile for visual UI animation
    FlyingBall projectile =
        new FlyingBall(
            shooterPos,
            landingPos,
            hubTarget,
            now,
            timeToHub,
            exitVelocity,
            LAUNCH_ANGLE_RAD,
            ballVx,
            ballVy,
            v0z,
            hit);
    activeFlyingBalls.add(projectile);
  }

  /** Updates all active ball projectile animations and removes expired ones. */
  private void updateFlyingBalls(double now) {
    synchronized (activeFlyingBalls) {
      Iterator<FlyingBall> iterator = activeFlyingBalls.iterator();
      while (iterator.hasNext()) {
        FlyingBall ball = iterator.next();
        if (now - ball.startTime > ball.flightDuration + 0.5) {
          iterator.remove();
        }
      }
    }
  }

  /** Computes the 2D field position of the shooter accounting for robot offset and rotation. */
  public Translation2d getShooterPosition(Pose2d robotPose) {
    Translation2d offset =
        kShooterOffsetFromRobotCenter.getTranslation().rotateBy(robotPose.getRotation());
    return robotPose.getTranslation().plus(offset);
  }

  /** Returns target Hub center based on active alliance. */
  public Translation2d getTargetHubCenter() {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    if (alliance == Alliance.Red) {
      return Hub.redCenter;
    } else {
      return Hub.blueCenter;
    }
  }

  /**
   * Evaluates if a theoretical shot from the current robot pose with given RPM will hit the target
   * Hub. Returns a TrajectoryPrediction record with arc coordinates and hit result.
   */
  public TrajectoryPrediction predictTrajectory(Pose2d robotPose, double currentRPM) {
    Translation2d shooterPos = getShooterPosition(robotPose);
    Translation2d hubTarget = getTargetHubCenter();
    double distance = shooterPos.getDistance(hubTarget);

    double exitVelocity = currentRPM * RPM_TO_EXIT_VELOCITY;
    double v0h = exitVelocity * Math.cos(LAUNCH_ANGLE_RAD);
    double v0z = exitVelocity * Math.sin(LAUNCH_ANGLE_RAD);

    double heading = robotPose.getRotation().getRadians();
    double ballVx = v0h * Math.cos(heading);
    double ballVy = v0h * Math.sin(heading);

    double deltaZ = HUB_HEIGHT_M - SHOOTER_HEIGHT_M;
    double discriminant = v0z * v0z - 2 * GRAVITY * deltaZ;

    boolean willHit = false;
    double timeToHub = 0.0;
    Translation2d landingPos = shooterPos;
    double landingError = Double.POSITIVE_INFINITY;

    if (discriminant >= 0 && exitVelocity > 1.0) {
      timeToHub = (v0z + Math.sqrt(discriminant)) / GRAVITY;
      double landX = shooterPos.getX() + ballVx * timeToHub;
      double landY = shooterPos.getY() + ballVy * timeToHub;
      landingPos = new Translation2d(landX, landY);
      landingError = landingPos.getDistance(hubTarget);
      if (landingError <= HUB_TARGET_RADIUS_M) {
        willHit = true;
      }
    }

    // Aim angle alignment
    Translation2d toHub = hubTarget.minus(shooterPos);
    double targetAngleRad = Math.atan2(toHub.getY(), toHub.getX());
    double angleErrorRad = Math.abs(Math.IEEEremainder(heading - targetAngleRad, 2 * Math.PI));

    return new TrajectoryPrediction(
        shooterPos,
        hubTarget,
        landingPos,
        distance,
        exitVelocity,
        v0h,
        v0z,
        timeToHub,
        landingError,
        angleErrorRad,
        willHit);
  }

  // Getters & Mutators for UI & Students
  public int getBallsInRobot() {
    return ballsInRobot;
  }

  public void setBallsInRobot(int count) {
    this.ballsInRobot = Math.max(0, Math.min(MAX_CAPACITY, count));
  }

  public void addBall() {
    if (ballsInRobot < MAX_CAPACITY) {
      ballsInRobot++;
    }
  }

  public void removeBall() {
    if (ballsInRobot > 0) {
      ballsInRobot--;
    }
  }

  public int getBallsScoredInHub() {
    return ballsScoredInHub;
  }

  public void resetBallsScored() {
    this.ballsScoredInHub = 0;
    this.totalShotsAttempted = 0;
    this.totalShotsMissed = 0;
  }

  public int getTotalShotsAttempted() {
    return totalShotsAttempted;
  }

  public int getTotalShotsMissed() {
    return totalShotsMissed;
  }

  public boolean hasBalls() {
    return ballsInRobot > 0;
  }

  public boolean isFull() {
    return ballsInRobot >= MAX_CAPACITY;
  }

  public boolean isLastShotSuccess() {
    return lastShotSuccess;
  }

  public double getLastShotDistance() {
    return lastShotDistance;
  }

  public List<FlyingBall> getActiveFlyingBalls() {
    return activeFlyingBalls;
  }

  /** Data record representing an in-flight simulated ball projectile. */
  public static class FlyingBall {
    public final Translation2d startPos;
    public final Translation2d targetLandPos;
    public final Translation2d hubCenter;
    public final double startTime;
    public final double flightDuration;
    public final double exitVelocity;
    public final double launchAngleRad;
    public final double vx;
    public final double vy;
    public final double vz0;
    public final boolean isHit;

    public FlyingBall(
        Translation2d startPos,
        Translation2d targetLandPos,
        Translation2d hubCenter,
        double startTime,
        double flightDuration,
        double exitVelocity,
        double launchAngleRad,
        double vx,
        double vy,
        double vz0,
        boolean isHit) {
      this.startPos = startPos;
      this.targetLandPos = targetLandPos;
      this.hubCenter = hubCenter;
      this.startTime = startTime;
      this.flightDuration = Math.max(0.1, flightDuration);
      this.exitVelocity = exitVelocity;
      this.launchAngleRad = launchAngleRad;
      this.vx = vx;
      this.vy = vy;
      this.vz0 = vz0;
      this.isHit = isHit;
    }

    /** Returns current 3D position [x, y, z] at elapsed time t. */
    public double[] getPositionAtTime(double currentTime) {
      double t = Math.max(0.0, currentTime - startTime);
      double x = startPos.getX() + vx * t;
      double y = startPos.getY() + vy * t;
      double z = SHOOTER_HEIGHT_M + vz0 * t - 0.5 * GRAVITY * t * t;
      return new double[] {x, y, Math.max(0.0, z)};
    }
  }

  /** Trajectory prediction summary for real-time visualization. */
  public record TrajectoryPrediction(
      Translation2d shooterPos,
      Translation2d hubTarget,
      Translation2d landingPos,
      double distanceToHub,
      double exitVelocity,
      double v0h,
      double v0z,
      double timeToHub,
      double landingErrorMeters,
      double angleErrorRad,
      boolean willHit) {}
}
