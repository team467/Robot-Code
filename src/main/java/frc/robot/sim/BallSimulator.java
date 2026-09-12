package frc.robot.sim;

import static frc.robot.subsystems.shooter.ShooterConstants.kShooterOffsetFromRobotCenter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.FieldConstants;
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

  public static final int MAX_CAPACITY = 20;
  public static final double GRAVITY = 9.81; // m/s^2
  public static final double LAUNCH_ANGLE_RAD = Math.toRadians(65.0); // 65 deg hood angle
  public static final double SHOOTER_HEIGHT_M = 0.50; // Initial release height off the carpet
  public static final double HUB_HEIGHT_M = Hub.height; // 1.8288 m (72 inches)
  public static final double HUB_TARGET_RADIUS_M = Hub.width / 2.0; // ~0.597 m (23.5 in radius)
  public static final double SHOT_INTERVAL_SECONDS = 0.28; // Min time between successive balls fed
  public static final double RPM_TO_EXIT_VELOCITY = 0.00525; // Calibrated launch velocity per RPM
  public static final double INTAKE_COOLDOWN_SECONDS =
      0.28; // Min time between successive intaked balls

  private int ballsInRobot = 8;
  private int ballsScoredInHub = 0;
  private int totalShotsAttempted = 0;
  private int totalShotsMissed = 0;

  private double lastShotTimestamp = 0.0;
  private boolean lastShotSuccess = false;
  private double lastShotDistance = 0.0;
  private double lastIntakeTimestamp = 0.0;

  private final Random random = new Random();
  private final List<FlyingBall> activeFlyingBalls =
      Collections.synchronizedList(new ArrayList<>());

  private final List<FieldBall> fieldBalls = Collections.synchronizedList(new ArrayList<>());

  public static synchronized BallSimulator getInstance() {
    if (instance == null) {
      instance = new BallSimulator();
    }
    return instance;
  }

  private BallSimulator() {
    initFieldBalls();
  }

  /** Initializes all field balls according to the 2026 field layout diagram. */
  public void initFieldBalls() {
    synchronized (fieldBalls) {
      fieldBalls.clear();

      double fieldL = FieldConstants.fieldLength;
      double fieldW = FieldConstants.fieldWidth;
      double xc = fieldL / 2.0;

      // 1. Center Neutral Zone Grid (Two symmetrical blocks across midline)
      double spacing = 0.20; // 20cm spacing between ball centers
      int cols = 12;
      int rows = 12;

      // Top block (upper half of neutral zone)
      double topStartY = fieldW / 2.0 + 0.35;
      for (int c = 0; c < cols; c++) {
        double bx = xc + (c - cols / 2.0 + 0.5) * spacing;
        for (int r = 0; r < rows; r++) {
          double by = topStartY + r * spacing;
          if (by < fieldW - 0.40) {
            fieldBalls.add(new FieldBall(bx, by));
          }
        }
      }

      // Bottom block (lower half of neutral zone)
      double bottomEndY = fieldW / 2.0 - 0.35;
      double bottomStartY = bottomEndY - (rows - 1) * spacing;
      for (int c = 0; c < cols; c++) {
        double bx = xc + (c - cols / 2.0 + 0.5) * spacing;
        for (int r = 0; r < rows; r++) {
          double by = bottomStartY + r * spacing;
          if (by > 0.40) {
            fieldBalls.add(new FieldBall(bx, by));
          }
        }
      }

      // 2. Red Wall Depots (Left side)
      // Upper depot (R1/R2 area)
      for (int c = 0; c < 3; c++) {
        for (int r = 0; r < 6; r++) {
          fieldBalls.add(new FieldBall(0.30 + c * spacing, 5.30 + r * spacing));
        }
      }
      // Lower depot (R3 corner)
      for (int c = 0; c < 2; c++) {
        for (int r = 0; r < 5; r++) {
          fieldBalls.add(new FieldBall(0.30 + c * spacing, 0.45 + r * spacing));
        }
      }

      // 3. Blue Wall Depots (Right side)
      // Lower depot (B1/B2 area)
      for (int c = 0; c < 3; c++) {
        for (int r = 0; r < 6; r++) {
          fieldBalls.add(new FieldBall(fieldL - 0.30 - c * spacing, 2.00 + r * spacing));
        }
      }
      // Upper depot (B3 corner)
      for (int c = 0; c < 2; c++) {
        for (int r = 0; r < 5; r++) {
          fieldBalls.add(new FieldBall(fieldL - 0.30 - c * spacing, fieldW - 0.45 - r * spacing));
        }
      }
    }
  }

  /** Reset all field balls back to their starting positions and zero velocity */
  public void resetFieldBalls() {
    lastIntakeTimestamp = 0.0;
    synchronized (fieldBalls) {
      for (FieldBall ball : fieldBalls) {
        ball.reset();
      }
    }
  }

  public List<FieldBall> getFieldBalls() {
    return fieldBalls;
  }

  /**
   * Main periodic update method called during simulation to evaluate intake, collisions, and
   * shooting physics.
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
    double fieldL = FieldConstants.fieldLength;
    double fieldW = FieldConstants.fieldWidth;

    // When disabled, only update physics of rolling balls and active flying balls
    if (DriverStation.isDisabled()) {
      synchronized (fieldBalls) {
        for (FieldBall ball : fieldBalls) {
          ball.update(0.02, fieldL, fieldW);
        }
      }
      updateFlyingBalls(now);
      return;
    }

    Pose2d robotPose = drive.getPose();
    double rx0 = robotPose.getX();
    double ry0 = robotPose.getY();
    Rotation2d rot = robotPose.getRotation();
    double cosTheta = rot.getCos();
    double sinTheta = rot.getSin();

    // Robot chassis velocities
    ChassisSpeeds speeds = drive.getChassisSpeeds();
    double vxRobotRel = speeds.vxMetersPerSecond;
    double vyRobotRel = speeds.vyMetersPerSecond;
    // Field-relative robot velocity
    double vRx = vxRobotRel * cosTheta - vyRobotRel * sinTheta;
    double vRy = vxRobotRel * sinTheta + vyRobotRel * cosTheta;
    double robotSpeed = Math.hypot(vRx, vRy);

    // 1. Process Physical Ball Interactions (Front Intake & Bumper Knocking)
    boolean isIntakeRunning = RobotState.getInstance().intaking;
    boolean isIntakeDeployed = RobotState.getInstance().intakePosition == IntakePosition.DEPLOYED;
    boolean isDrivingForward = vxRobotRel > 0.08;
    boolean canIntake =
        isIntakeRunning && isIntakeDeployed && isDrivingForward && ballsInRobot < MAX_CAPACITY;

    boolean ballIntakedThisTick = false;
    double bumperHl = isIntakeDeployed ? 0.58 : 0.44; // Front extends when deployed
    double bumperHw = 0.44;
    double ballR = FieldBall.RADIUS;

    synchronized (fieldBalls) {
      for (FieldBall ball : fieldBalls) {
        if (!ball.inPlay) continue;

        // Broadphase distance check from robot center
        double dx = ball.x - rx0;
        double dy = ball.y - ry0;
        double dist = Math.hypot(dx, dy);
        if (dist > 1.3) {
          ball.update(0.02, fieldL, fieldW);
          continue;
        }

        // Transform into robot-relative coordinates (+rx = forward, +ry = left)
        double rx = dx * cosTheta + dy * sinTheta;
        double ry = -dx * sinTheta + dy * cosTheta;

        // A. Front Intake Collection Zone
        if (canIntake && rx >= 0.35 && rx <= 0.72 && Math.abs(ry) <= 0.38) {
          // Intake can only take one ball at a time and requires a minimum interval between pickups
          boolean intakeReady =
              !ballIntakedThisTick && (now - lastIntakeTimestamp >= INTAKE_COOLDOWN_SECONDS);

          // Realistic capture rate: balls hitting center roller have ~65% grab rate,
          // outer edges have ~35% grab rate; remaining balls bounce off randomly
          double grabRate = (Math.abs(ry) > 0.25) ? 0.35 : 0.65;
          boolean successfullyGrabbed = intakeReady && (random.nextDouble() < grabRate);

          if (successfullyGrabbed) {
            ball.inPlay = false;
            ball.vx = 0.0;
            ball.vy = 0.0;
            ballsInRobot++;
            ballIntakedThisTick = true;
            lastIntakeTimestamp = now;
            Logger.recordOutput("BallSim/IntakeEvent", true);
            continue;
          } else {
            // Ball bounces off the intake roller randomly
            double lateralSign =
                (Math.abs(ry) > 0.04) ? Math.signum(ry) : (random.nextBoolean() ? 1.0 : -1.0);
            double lateralVel = lateralSign * (0.8 + random.nextDouble() * 1.2);
            double forwardVel = Math.max(0.6, robotSpeed * 0.7) + random.nextDouble() * 0.9;

            // Transform bounce velocity to field coordinates
            ball.vx = vRx + forwardVel * cosTheta - lateralVel * sinTheta;
            ball.vy = vRy + forwardVel * sinTheta + lateralVel * cosTheta;

            // Displace ball slightly forward/outward outside of immediate intake grasp
            double pushRx = 0.76 + random.nextDouble() * 0.06;
            double pushRy = ry + lateralSign * 0.06;
            ball.x = rx0 + pushRx * cosTheta - pushRy * sinTheta;
            ball.y = ry0 + pushRx * sinTheta + pushRy * cosTheta;

            ball.update(0.02, fieldL, fieldW);
            continue;
          }
        }

        // B. Robot Bumper Collision (Knock & Push)
        if (rx > -0.44 - ballR && rx < bumperHl + ballR && Math.abs(ry) < bumperHw + ballR) {
          // Push out along minimum penetration axis
          double penX = (rx > 0) ? (bumperHl + ballR - rx) : (-0.44 - ballR - rx);
          double penY = (ry > 0) ? (bumperHw + ballR - ry) : (-bumperHw - ballR - ry);

          double normX = 0;
          double normY = 0;
          if (Math.abs(penX) < Math.abs(penY)) {
            normX = Math.signum(rx);
            rx += normX * Math.abs(penX);
          } else {
            normY = Math.signum(ry);
            ry += normY * Math.abs(penY);
          }

          // Transform back to field frame
          double fnx = normX * cosTheta - normY * sinTheta;
          double fny = normX * sinTheta + normY * cosTheta;
          ball.x = rx0 + rx * cosTheta - ry * sinTheta;
          ball.y = ry0 + rx * sinTheta + ry * cosTheta;

          // Impart momentum from robot chassis + kick impulse
          double kick = Math.max(robotSpeed * 1.3, 0.4);
          ball.vx = vRx + fnx * kick;
          ball.vy = vRy + fny * kick;
        }

        ball.update(0.02, fieldL, fieldW);
      }

      // Ball-to-ball dispersal for moving balls
      for (FieldBall b1 : fieldBalls) {
        if (!b1.inPlay) continue;
        double s1 = Math.hypot(b1.vx, b1.vy);
        if (s1 < 0.15) continue; // Only moving balls push other balls

        for (FieldBall b2 : fieldBalls) {
          if (!b2.inPlay || b1 == b2) continue;
          double bdx = b2.x - b1.x;
          double bdy = b2.y - b1.y;
          double bDist = Math.hypot(bdx, bdy);
          double minDist = FieldBall.RADIUS * 2.0;
          if (bDist < minDist && bDist > 0.001) {
            double nx = bdx / bDist;
            double ny = bdy / bDist;
            double overlap = (minDist - bDist) * 0.5;
            b2.x += nx * overlap;
            b2.y += ny * overlap;
            b1.x -= nx * overlap;
            b1.y -= ny * overlap;

            double impulse = (b1.vx * nx + b1.vy * ny) * 0.6;
            if (impulse > 0) {
              b2.vx += nx * impulse;
              b2.vy += ny * impulse;
              b1.vx -= nx * impulse * 0.5;
              b1.vy -= ny * impulse * 0.5;
            }
          }
        }
      }
    }

    // 2. Process Shooting Logic (only fire when shooter is up to speed)
    boolean isShooterRunning = RobotState.getInstance().shooterAtSpeed;
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
      // Ball can only score if shot from the robot's own alliance zone (backboard blocks
      // neutral/opponent zone shots)
      if (landingError <= HUB_TARGET_RADIUS_M && isInAllianceZone(robotPose)) {
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
        double elapsed = now - ball.startTime;
        // Balls that score disappear when they land in the hub
        if (ball.isHit && elapsed >= ball.flightDuration) {
          iterator.remove();
        } else if (elapsed > ball.flightDuration + 0.5) {
          iterator.remove();
        }
      }
    }
  }

  /** Checks if the robot is located in its own alliance zone (outside neutral zone). */
  public boolean isInAllianceZone(Pose2d robotPose) {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    if (alliance == Alliance.Red) {
      return robotPose.getX() > LinesVertical.neutralZoneFar;
    } else {
      return robotPose.getX() < LinesVertical.neutralZoneNear;
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
      if (landingError <= HUB_TARGET_RADIUS_M && isInAllianceZone(robotPose)) {
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
    resetFieldBalls();
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

  /** Represents a physical 2D ball on the field carpet with collision and rolling physics. */
  public static class FieldBall {
    public static final double RADIUS = 0.09; // ~7 inch diameter ball (0.09m radius)
    public static final double RESTITUTION = 0.45; // Wall bounce elasticity
    public static final double CARPET_FRICTION_DAMPING = 0.93; // Velocity decay per 20ms frame
    public static final double STOP_VELOCITY_THRESHOLD = 0.03; // m/s below which ball stops

    public double x;
    public double y;
    public double vx;
    public double vy;
    public boolean inPlay;

    public final double homeX;
    public final double homeY;

    public FieldBall(double x, double y) {
      this.x = x;
      this.y = y;
      this.homeX = x;
      this.homeY = y;
      this.vx = 0.0;
      this.vy = 0.0;
      this.inPlay = true;
    }

    public void reset() {
      this.x = homeX;
      this.y = homeY;
      this.vx = 0.0;
      this.vy = 0.0;
      this.inPlay = true;
    }

    public void update(double dt, double fieldL, double fieldW) {
      if (!inPlay) return;

      if (Math.abs(vx) > 0.001 || Math.abs(vy) > 0.001) {
        x += vx * dt;
        y += vy * dt;

        // Carpet rolling friction damping
        vx *= CARPET_FRICTION_DAMPING;
        vy *= CARPET_FRICTION_DAMPING;

        if (Math.hypot(vx, vy) < STOP_VELOCITY_THRESHOLD) {
          vx = 0.0;
          vy = 0.0;
        }

        // Field wall collisions
        if (x < RADIUS) {
          x = RADIUS;
          vx = -vx * RESTITUTION;
        } else if (x > fieldL - RADIUS) {
          x = fieldL - RADIUS;
          vx = -vx * RESTITUTION;
        }

        if (y < RADIUS) {
          y = RADIUS;
          vy = -vy * RESTITUTION;
        } else if (y > fieldW - RADIUS) {
          y = fieldW - RADIUS;
          vy = -vy * RESTITUTION;
        }
      }
    }
  }
}
