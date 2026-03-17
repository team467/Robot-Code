/*
 * ShotCalculator.java - Newton-method SOTM fire control with drag compensation
 *
 * MIT License
 *
 * Copyright (c) 2026 FRC Team 5962 perSEVERE
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.
 */

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Shoot-on-the-move fire control solver. Figures out what RPM and heading your robot needs while
 * you're driving around. It accounts for robot velocity, where the launcher is on the robot,
 * processing latency, and drag on the ball during flight.
 *
 * <p>The core idea: if you're moving, you can't just aim at the target because the ball inherits
 * your velocity. So we use Newton's method to find the self-consistent time-of-flight where the
 * projected aim point and the LUT-predicted TOF agree. Usually converges in 2-3 iterations.
 *
 * <p>Usage:
 *
 * <pre>
 *   // configure for your robot (measure from CAD)
 *   ShotCalculator.Config config = new ShotCalculator.Config();
 *   config.launcherOffsetX = 0.23;  // meters forward of robot center
 *   config.launcherOffsetY = 0.0;   // meters left of center
 *
 *   ShotCalculator calc = new ShotCalculator(config);
 *
 *   // load your shooter LUT (from ProjectileSimulator or hand-tuned)
 *   for (var entry : lut.entries()) {
 *       if (entry.reachable()) {
 *           calc.loadLUTEntry(entry.distanceM(), entry.rpm(), entry.tof());
 *       }
 *   }
 *
 *   // call once per robot cycle
 *   ShotCalculator.ShotInputs inputs = new ShotCalculator.ShotInputs(
 *       swerve.getPose(), swerve.getFieldVelocity(), swerve.getRobotVelocity(),
 *       hubCenter, hubForwardVector, visionConfidence
 *   );
 *   ShotCalculator.LaunchParameters result = calc.calculate(inputs);
 *   if (result.isValid() &amp;&amp; result.confidence() &gt; 50) {
 *       shooter.setRPM(result.rpm());
 *       drivebase.setHeading(result.driveAngle());
 *   }
 * </pre>
 */
public class ShotCalculator {

  /**
   * The result of calculate(). RPM to spin up, time of flight, heading to aim at, and a 0-100
   * confidence score.
   */
  public record LaunchParameters(
      double rpm,
      double timeOfFlightSec,
      Rotation2d driveAngle,
      double driveAngularVelocityRadPerSec,
      boolean isValid,
      double confidence,
      double solvedDistanceM,
      int iterationsUsed,
      boolean warmStartUsed) {

    public static final LaunchParameters INVALID =
        new LaunchParameters(0, 0, new Rotation2d(), 0, false, 0, 0, 0, false);
  }

  /**
   * All the state the solver needs from your robot each cycle. pitchDeg and rollDeg are absolute
   * tilt angles in degrees. If your gyro doesn't report these, just pass 0.0 for both and set
   * config.maxTiltDeg to something huge.
   */
  public record ShotInputs(
      Pose2d robotPose,
      ChassisSpeeds fieldVelocity,
      ChassisSpeeds robotVelocity,
      Translation2d hubCenter,
      Translation2d hubForward,
      double visionConfidence,
      double pitchDeg,
      double rollDeg) {

    /** Convenience constructor for callers that don't have pitch/roll data. */
    public ShotInputs(
        Pose2d robotPose,
        ChassisSpeeds fieldVelocity,
        ChassisSpeeds robotVelocity,
        Translation2d hubCenter,
        Translation2d hubForward,
        double visionConfidence) {
      this(
          robotPose,
          fieldVelocity,
          robotVelocity,
          hubCenter,
          hubForward,
          visionConfidence,
          0.0,
          0.0);
    }
  }

  /**
   * Tuning parameters. Set these to match your robot, or wire them to SmartDashboard/TunableNumber.
   */
  public static class Config {
    // Launcher geometry (measure from CAD)
    public double launcherOffsetX = 0.20; // meters forward of robot center
    public double launcherOffsetY = 0.0; // meters left of robot center

    // How close/far you can score from (meters)
    public double minScoringDistance = 0.5;
    public double maxScoringDistance = 5.0;

    // Newton solver tuning
    public int maxIterations = 25;
    public double convergenceTolerance = 0.001; // seconds
    public double tofMin = 0.05;
    public double tofMax = 5.0;

    // Below this speed (m/s), don't bother with SOTM, just aim straight
    public double minSOTMSpeed = 0.1;

    // Above this speed (m/s), don't shoot, we're outside calibration range
    public double maxSOTMSpeed = 3.0;

    // Latency compensation (ms)
    public double phaseDelayMs = 30.0; // vision pipeline lag
    public double mechLatencyMs = 20.0; // how long the mechanism takes to respond

    // The ball's inherited robot velocity decays in flight because of drag.
    // Real displacement = (1 - e^(-c*tof)) / c instead of just v*tof.
    // Set to 0 to disable drag compensation.
    public double sotmDragCoeff = 0.47;

    // Confidence scoring weights (5-component weighted geometric mean)
    public double wConvergence = 1.0;
    public double wVelocityStability = 0.8;
    public double wVisionConfidence = 1.2;
    public double wHeadingAccuracy = 1.5;
    public double wDistanceInRange = 0.5;
    public double headingMaxErrorRad = Math.toRadians(15);

    // Heading tolerance tightens as robot speed increases.
    // scaledMaxError = base / (1 + speedScalar * speed). Set to 0 to disable.
    public double headingSpeedScalar = 1.0;

    // Heading tolerance scales with distance from hub.
    // Closer = tighter because small angle errors matter more up close.
    // scaledMaxError *= referenceDistance / distance, clamped [0.5, 2.0].
    public double headingReferenceDistance = 2.5; // meters

    // Suppress firing when pitch or roll exceeds this threshold.
    // Bumps and ramps tilt the robot, which throws off aim. Set to 90 to disable.
    public double maxTiltDeg = 5.0;
  }

  private final Config config;

  private final InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap tofMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap correctionRpmMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap correctionTofMap = new InterpolatingDoubleTreeMap();

  // Copilot RPM trim (flat offset applied during match)
  private double rpmOffset = 0;

  // Solver state (reused across cycles to avoid allocation)
  private double previousTOF = -1;
  private double previousSpeed = 0;

  // Previous-cycle velocities for acceleration estimation
  private double prevRobotVx = 0;
  private double prevRobotVy = 0;
  private double prevRobotOmega = 0;

  public ShotCalculator(Config config) {
    this.config = config;
  }

  /** Default config. You still need to call loadLUTEntry() to fill the lookup tables. */
  public ShotCalculator() {
    this(new Config());
  }

  /**
   * Add a distance/RPM/TOF point to the lookup table. Use ProjectileSimulator to generate these, or
   * hand-tune.
   */
  public void loadLUTEntry(double distanceM, double rpm, double tof) {
    rpmMap.put(distanceM, rpm);
    tofMap.put(distanceM, tof);
  }

  // LUT lookup: base value + any corrections + copilot RPM offset
  double effectiveRPM(double distance) {
    double base = rpmMap.get(distance);
    Double correction = correctionRpmMap.get(distance);
    return base + (correction != null ? correction : 0.0) + rpmOffset;
  }

  double effectiveTOF(double distance) {
    double base = tofMap.get(distance);
    Double correction = correctionTofMap.get(distance);
    return base + (correction != null ? correction : 0.0);
  }

  // Drag-adjusted effective TOF: actual displacement < v*tof because drag.
  // Returns (1 - e^(-c*tof)) / c, or just tof if no drag.
  private double dragCompensatedTOF(double tof) {
    double c = config.sotmDragCoeff;
    if (c < 1e-6) return tof; // no drag correction
    return (1.0 - Math.exp(-c * tof)) / c;
  }

  /** Central finite difference derivative of the TOF lookup table. */
  private static final double DERIV_H = 0.01; // 1cm step

  double tofMapDerivative(double d) {
    double tHigh = effectiveTOF(d + DERIV_H);
    double tLow = effectiveTOF(d - DERIV_H);
    return (tHigh - tLow) / (2.0 * DERIV_H);
  }

  /**
   * Solve for the firing solution. Call once per cycle in robotPeriodic(). Returns INVALID if
   * you're out of range, behind the hub, going too fast, or the inputs are bad.
   */
  public LaunchParameters calculate(ShotInputs inputs) {
    if (inputs == null
        || inputs.robotPose() == null
        || inputs.fieldVelocity() == null
        || inputs.robotVelocity() == null) {
      return LaunchParameters.INVALID;
    }

    Pose2d rawPose = inputs.robotPose();
    ChassisSpeeds fieldVel = inputs.fieldVelocity();
    ChassisSpeeds robotVel = inputs.robotVelocity();

    double poseX = rawPose.getX();
    double poseY = rawPose.getY();
    if (Double.isNaN(poseX)
        || Double.isNaN(poseY)
        || Double.isInfinite(poseX)
        || Double.isInfinite(poseY)) {
      return LaunchParameters.INVALID;
    }

    // Second-order pose prediction. Instead of just v*dt, we use v*dt + 0.5*a*dt^2
    // where acceleration is estimated from the velocity delta between this cycle and last.
    // This tracks better through turns and speed changes because it catches the curvature.
    double dt = config.phaseDelayMs / 1000.0;
    double ax = (robotVel.vxMetersPerSecond - prevRobotVx) / 0.02;
    double ay = (robotVel.vyMetersPerSecond - prevRobotVy) / 0.02;
    double aOmega = (robotVel.omegaRadiansPerSecond - prevRobotOmega) / 0.02;
    Pose2d compensatedPose =
        rawPose.exp(
            new Twist2d(
                robotVel.vxMetersPerSecond * dt + 0.5 * ax * dt * dt,
                robotVel.vyMetersPerSecond * dt + 0.5 * ay * dt * dt,
                robotVel.omegaRadiansPerSecond * dt + 0.5 * aOmega * dt * dt));
    prevRobotVx = robotVel.vxMetersPerSecond;
    prevRobotVy = robotVel.vyMetersPerSecond;
    prevRobotOmega = robotVel.omegaRadiansPerSecond;

    double robotX = compensatedPose.getX();
    double robotY = compensatedPose.getY();
    double heading = compensatedPose.getRotation().getRadians();

    Translation2d hubCenter = inputs.hubCenter();
    double hubX = hubCenter.getX();
    double hubY = hubCenter.getY();

    // Behind-hub detection: dot product with hub forward vector
    Translation2d hubForward = inputs.hubForward();
    double dot = (hubX - robotX) * hubForward.getX() + (hubY - robotY) * hubForward.getY();
    if (dot < 0) {
      return LaunchParameters.INVALID;
    }

    // Tilt gate. Bumps and ramps knock the launcher off-axis, so
    // suppress firing when the chassis is tilted beyond the threshold.
    if (Math.abs(inputs.pitchDeg()) > config.maxTiltDeg
        || Math.abs(inputs.rollDeg()) > config.maxTiltDeg) {
      return LaunchParameters.INVALID;
    }

    // Transform robot center to launcher position
    double cosH = Math.cos(heading);
    double sinH = Math.sin(heading);
    double launcherX = robotX + config.launcherOffsetX * cosH - config.launcherOffsetY * sinH;
    double launcherY = robotY + config.launcherOffsetX * sinH + config.launcherOffsetY * cosH;

    // Launcher velocity includes rotational component: v_launcher = v_robot + omega x r
    double launcherFieldOffX = config.launcherOffsetX * cosH - config.launcherOffsetY * sinH;
    double launcherFieldOffY = config.launcherOffsetX * sinH + config.launcherOffsetY * cosH;
    double omega = fieldVel.omegaRadiansPerSecond;
    double vx = fieldVel.vxMetersPerSecond + (-launcherFieldOffY) * omega;
    double vy = fieldVel.vyMetersPerSecond + launcherFieldOffX * omega;

    // Displacement from launcher to hub
    double rx = hubX - launcherX;
    double ry = hubY - launcherY;
    double distance = Math.hypot(rx, ry);

    if (distance < config.minScoringDistance || distance > config.maxScoringDistance) {
      return LaunchParameters.INVALID;
    }

    double robotSpeed = Math.hypot(vx, vy);

    // Speed cap: shots above this speed are out of calibration range
    if (robotSpeed > config.maxSOTMSpeed) {
      return LaunchParameters.INVALID;
    }

    boolean velocityFiltered = robotSpeed < config.minSOTMSpeed;

    double solvedTOF;
    double projDist;
    int iterationsUsed;
    boolean warmStartUsed;

    if (velocityFiltered) {
      // Static shot: no velocity compensation needed
      solvedTOF = effectiveTOF(distance);
      projDist = distance;
      iterationsUsed = 0;
      warmStartUsed = false;
    } else {
      // Newton-method SOTM solver
      int maxIter = config.maxIterations;
      double convTol = config.convergenceTolerance;

      // Warm start from previous cycle's solution when available
      double tof;
      if (previousTOF > 0) {
        tof = previousTOF;
        warmStartUsed = true;
      } else {
        tof = effectiveTOF(distance);
        warmStartUsed = false;
      }

      projDist = distance;
      iterationsUsed = 0;

      for (int i = 0; i < maxIter; i++) {
        double prevTOF = tof;

        // Projected displacement at time t, with drag-compensated velocity offset
        double driftTOF = dragCompensatedTOF(tof);
        double prx = rx - vx * driftTOF;
        double pry = ry - vy * driftTOF;
        projDist = Math.hypot(prx, pry);

        // Degenerate guard: ball is essentially on top of the hub
        if (projDist < 0.01) {
          tof = effectiveTOF(distance);
          iterationsUsed = maxIter + 1; // flag as diverged
          break;
        }

        double lookupTOF = effectiveTOF(projDist);

        // Derivative for Newton step
        double dPrime = -(prx * vx + pry * vy) / projDist;
        double gPrime = tofMapDerivative(projDist);
        double f = lookupTOF - tof;
        double fPrime = gPrime * dPrime - 1.0;

        // Newton step with near-zero denominator guard
        if (Math.abs(fPrime) > 0.01) {
          tof = tof - f / fPrime;
        } else {
          tof = lookupTOF; // fixed-point fallback
        }

        // Per-iteration clamp prevents runaway
        tof = MathUtil.clamp(tof, config.tofMin, config.tofMax);

        iterationsUsed = i + 1;

        // Convergence check
        if (Math.abs(tof - prevTOF) < convTol) {
          break;
        }
      }

      // Divergence guard
      if (tof > config.tofMax || tof < 0.0 || Double.isNaN(tof)) {
        tof = effectiveTOF(distance);
        iterationsUsed = maxIter + 1;
      }

      solvedTOF = tof;
    }

    // Save for next cycle's warm start
    previousTOF = solvedTOF;

    double effectiveTOF = solvedTOF + config.mechLatencyMs / 1000.0;

    // RPM from LUT at solved distance
    double effectiveRPMValue = effectiveRPM(projDist);

    // Drive angle: aim at velocity-compensated target position
    double compTargetX;
    double compTargetY;
    if (velocityFiltered) {
      compTargetX = hubX;
      compTargetY = hubY;
    } else {
      double headingDriftTOF = dragCompensatedTOF(solvedTOF);
      compTargetX = hubX - vx * headingDriftTOF;
      compTargetY = hubY - vy * headingDriftTOF;
    }
    double aimX = compTargetX - robotX;
    double aimY = compTargetY - robotY;
    Rotation2d driveAngle = new Rotation2d(aimX, aimY);

    // Heading error for confidence calculation
    double headingErrorRad = MathUtil.angleModulus(driveAngle.getRadians() - heading);

    // Angular velocity feedforward: rate of change of aim angle
    double driveAngularVelocity = 0;
    if (!velocityFiltered && distance > 0.1) {
      // tangential velocity / distance gives angular rate
      double tangentialVel = (-ry * vx + rx * vy) / distance;
      driveAngularVelocity = tangentialVel / distance;
    }

    // Solver convergence quality
    double solverQuality;
    if (velocityFiltered) {
      solverQuality = 1.0;
    } else {
      int maxIter = config.maxIterations;
      if (iterationsUsed > maxIter) {
        solverQuality = 0.0;
      } else if (iterationsUsed <= 3) {
        solverQuality = 1.0;
      } else {
        solverQuality =
            MathUtil.interpolate(1.0, 0.1, (double) (iterationsUsed - 3) / (maxIter - 3));
      }
    }

    double confidence =
        computeConfidence(
            solverQuality, robotSpeed, headingErrorRad, distance, inputs.visionConfidence());

    previousSpeed = robotSpeed;

    return new LaunchParameters(
        effectiveRPMValue,
        effectiveTOF,
        driveAngle,
        driveAngularVelocity,
        true,
        confidence,
        distance,
        iterationsUsed,
        warmStartUsed);
  }

  /**
   * Confidence from 0 to 100. Weighted geometric mean of 5 factors: solver convergence, velocity
   * stability, vision confidence, heading accuracy, and distance from range edges. If any single
   * factor drops to zero (like vision dies), the whole score tanks to zero. That's intentional
   * because you really shouldn't be shooting if any one factor is gone.
   */
  private double computeConfidence(
      double solverQuality,
      double currentSpeed,
      double headingErrorRad,
      double distance,
      double visionConfidence) {

    // 1. Solver quality (passed in, already 0-1)
    double convergenceQuality = solverQuality;

    // 2. Velocity stability: penalize rapid speed changes
    double speedDelta = Math.abs(currentSpeed - previousSpeed);
    double velocityStability = MathUtil.clamp(1.0 - speedDelta / 0.5, 0, 1);

    // 3. Vision confidence (0-1, from caller)
    double visionConf = MathUtil.clamp(visionConfidence, 0, 1);

    // 4. Heading accuracy with speed scaling and distance scaling.
    // Faster robot = tighter tolerance (because velocity errors compound).
    // Closer to hub = tighter tolerance (because small angles mean big misses).
    double distanceScale = MathUtil.clamp(config.headingReferenceDistance / distance, 0.5, 2.0);
    double speedScale = 1.0 / (1.0 + config.headingSpeedScalar * currentSpeed);
    double scaledMaxError = config.headingMaxErrorRad * distanceScale * speedScale;
    double headingErr = Math.abs(headingErrorRad);
    double headingAccuracy = MathUtil.clamp(1.0 - headingErr / scaledMaxError, 0, 1);

    // 5. Distance in range: penalty for being near min/max scoring boundaries
    double rangeSpan = config.maxScoringDistance - config.minScoringDistance;
    double rangeFraction = (distance - config.minScoringDistance) / rangeSpan;
    double distInRange = 1.0 - 2.0 * Math.abs(rangeFraction - 0.5);
    distInRange = MathUtil.clamp(distInRange, 0, 1);

    // Weighted geometric mean (one zero kills it)
    double[] c = {convergenceQuality, velocityStability, visionConf, headingAccuracy, distInRange};
    double[] w = {
      config.wConvergence,
      config.wVelocityStability,
      config.wVisionConfidence,
      config.wHeadingAccuracy,
      config.wDistanceInRange
    };

    double sumW = 0;
    double logSum = 0;
    for (int i = 0; i < 5; i++) {
      if (c[i] <= 0) return 0;
      logSum += w[i] * Math.log(c[i]);
      sumW += w[i];
    }

    if (sumW <= 0) return 0;
    double composite = Math.exp(logSum / sumW) * 100.0;
    return MathUtil.clamp(composite, 0, 100);
  }

  /** Layer a per-distance RPM adjustment on top of the base LUT. Good for field tuning at comp. */
  public void addRpmCorrection(double distance, double deltaRpm) {
    correctionRpmMap.put(distance, deltaRpm);
  }

  /** Layer a per-distance TOF adjustment on top of the base LUT. */
  public void addTofCorrection(double distance, double deltaTof) {
    correctionTofMap.put(distance, deltaTof);
  }

  /** Clear all corrections, back to the raw LUT. */
  public void clearCorrections() {
    correctionRpmMap.clear();
    correctionTofMap.clear();
  }

  /** Bump the RPM offset by delta. Clamped to +/- 200. Bind this to copilot D-pad. */
  public void adjustOffset(double delta) {
    rpmOffset = MathUtil.clamp(rpmOffset + delta, -200, 200);
  }

  /** Reset the RPM offset to zero. Call this on mode transitions so trim doesn't carry over. */
  public void resetOffset() {
    rpmOffset = 0;
  }

  public double getOffset() {
    return rpmOffset;
  }

  /** Raw time-of-flight from the LUT at this distance (no velocity compensation). */
  public double getTimeOfFlight(double distanceM) {
    return effectiveTOF(distanceM);
  }

  /** Base RPM at this distance, before any corrections or offset. */
  public double getBaseRPM(double distance) {
    return rpmMap.get(distance);
  }

  /**
   * Reset the warm start state. Call this after a pose reset so the solver doesn't use stale data.
   */
  public void resetWarmStart() {
    previousTOF = -1;
    previousSpeed = 0;
    prevRobotVx = 0;
    prevRobotVy = 0;
    prevRobotOmega = 0;
  }

  InterpolatingDoubleTreeMap getRpmMap() {
    return rpmMap;
  }

  InterpolatingDoubleTreeMap getTofMap() {
    return tofMap;
  }
}
