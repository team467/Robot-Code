/*
 * FuelPhysicsSim.java - Full-field ball physics simulation for FRC 2026 REBUILT
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

package frc.robot.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/**
 * Full-field ball physics simulation for FRC 2026 REBUILT. Handles drag, Magnus lift, friction,
 * ball-ball collisions, wall bounces, hub scoring, sleeping, CCD, and robot interaction. Single
 * file, only depends on WPILib (wpimath + ntcore). Drop it into your sim and watch balls fly.
 *
 * <p>Physics: symplectic Euler integration, 3D angular velocity for Magnus (omega x v cross
 * product), Coulomb friction with spin transfer, sequential impulse collision solver with warm
 * starting and Baumgarte stabilization. Spatial hashing for ball-ball broadphase. Ball sleeping
 * keeps 350+ resting balls under 2ms/tick.
 *
 * <p>Usage:
 *
 * <pre>
 *   FuelPhysicsSim ballSim = new FuelPhysicsSim("Sim/Fuel");
 *   ballSim.enable();
 *   ballSim.placeFieldBalls();   // spawns all the game pieces
 *   // in simulationPeriodic():
 *   ballSim.configureRobot(width, length, bumperH, poseSupplier, speedsSupplier);
 *   ballSim.tick();              // runs physics, publishes to NT
 * </pre>
 */
public class FuelPhysicsSim {

  // Physics constants

  private static final double GRAVITY = 9.81; // m/s^2
  private static final double AIR_DENSITY = 1.225; // kg/m^3, standard atmosphere
  private static final double BALL_MASS = 0.215; // kg, game manual 5.10.1 midpoint
  private static final double BALL_DIAMETER = 0.1501; // m, game manual 5.10.1
  private static final double BALL_RADIUS = BALL_DIAMETER / 2.0;
  private static final double BALL_CROSS_AREA = Math.PI * BALL_RADIUS * BALL_RADIUS;
  private static final double BALL_MOMENT_OF_INERTIA =
      0.4 * BALL_MASS * BALL_RADIUS * BALL_RADIUS; // 2/5 * m * r^2, solid sphere

  // Aerodynamic coefficients
  private static final double DEFAULT_CD = 0.47; // drag coefficient, smooth sphere
  private static final double DEFAULT_CM = 0.2; // Magnus coefficient, conservative estimate

  // Precomputed force factors (divided by mass to get acceleration factors)
  private static final double DRAG_ACCEL_FACTOR =
      0.5 * AIR_DENSITY * DEFAULT_CD * BALL_CROSS_AREA / BALL_MASS;
  // Extra BALL_RADIUS factor converts the omega x v cross product to acceleration
  private static final double MAGNUS_ACCEL_FACTOR =
      0.5 * AIR_DENSITY * DEFAULT_CM * BALL_CROSS_AREA * BALL_RADIUS / BALL_MASS;

  // Coefficients of restitution (per-material, from field element build instructions)
  private static final double COR_CARPET = 0.65; // foam on low-pile carpet
  private static final double COR_WALL = 0.70; // foam on polycarbonate (alliance walls, guardrails)
  private static final double COR_STEEL = 0.72; // foam on powder-coated steel (tower, rungs)
  private static final double COR_HUB = 0.70; // foam on polycarbonate (hub body panels)
  private static final double COR_HDPE = 0.60; // foam on textured HDPE (bump ramps, 15deg)
  private static final double COR_NET = 0.15; // mesh fabric absorbs most of the energy
  private static final double COR_BUMPER = 0.08; // polycarb-backed foam, nearly inelastic
  private static final double COR_BALL_BALL = 0.45; // foam-on-foam, high deformation loss

  // Friction coefficients
  private static final double MU_GROUND_KINETIC = 0.3; // kinetic friction on carpet
  private static final double MU_GROUND_ROLLING = 0.05; // rolling friction
  private static final double MU_WALL = 0.4; // foam on polycarbonate/fabric
  private static final double MU_BALL_BALL = 0.3; // foam on foam

  // Velocity-dependent COR reference speed
  private static final double COR_VREF = 3.0; // m/s
  private static final double COR_EXPONENT = 0.15;

  // Reusable axis-aligned unit normals (avoids allocating these in hot loops)
  private static final Translation3d AXIS_X_POS = new Translation3d(1, 0, 0);
  private static final Translation3d AXIS_X_NEG = new Translation3d(-1, 0, 0);
  private static final Translation3d AXIS_Y_POS = new Translation3d(0, 1, 0);
  private static final Translation3d AXIS_Y_NEG = new Translation3d(0, -1, 0);
  private static final Translation3d AXIS_Z_POS = new Translation3d(0, 0, 1);
  private static final Translation3d AXIS_Z_NEG = new Translation3d(0, 0, -1);

  // Period
  private static final double PERIOD = 0.02; // 20ms

  // Field geometry

  private static final double FIELD_LENGTH = 16.541; // m
  private static final double FIELD_WIDTH = 8.052; // m

  // Alliance wall and guardrail heights
  private static final double ALLIANCE_WALL_HEIGHT = 0.935; // 36.8 in
  private static final double GUARDRAIL_HEIGHT = 0.508; // 20 in

  // Bump geometry (tent-shaped, 15-degree ramps)
  private static final double BUMP_HEIGHT = 0.165; // m, 6.513 in

  // Hub positions (from game manual field layout)
  private static final Translation2d BLUE_HUB_CENTER = new Translation2d(4.5974, 4.035);
  private static final Translation2d RED_HUB_CENTER = new Translation2d(11.938, 4.035);

  // Hub dimensions
  private static final double HUB_ENTRY_HEIGHT = 1.829; // m, 72 in
  private static final double HUB_ENTRY_RADIUS = 0.5295; // m, 41.7 in / 2 across flats
  private static final double HUB_SIDE = 1.194; // m, 47 in square base
  // Hub base structure height for ball collision (not a solid wall to scoring height,
  // just the base frame that deflects ground-level balls)
  private static final double HUB_BASE_HEIGHT = 0.5; // m, ~20 in base frame

  // Net dimensions
  private static final double NET_HEIGHT_MIN = 1.5; // m
  private static final double NET_HEIGHT_MAX = 3.057; // m
  private static final double NET_WIDTH = 1.484; // m
  private static final double NET_OFFSET = HUB_SIDE / 2.0 + 0.261;

  // Trench geometry
  private static final double TRENCH_WIDTH = 1.265; // m
  private static final double TRENCH_BLOCK_WIDTH = 0.305; // m, 12 in
  private static final double TRENCH_HEIGHT = 0.565; // m, 22.25 in underpass
  private static final double TRENCH_PILLAR_HEIGHT = 1.346; // m, 53 in

  // Tower geometry
  private static final double TOWER_POLE_WIDTH = 0.051; // m, 2 in
  private static final double TOWER_POLE_HEIGHT = 1.194; // m, 47 in
  private static final double TOWER_UPRIGHT_THICK = 0.038; // m, 1.5 in
  private static final double TOWER_UPRIGHT_DEPTH = 0.089; // m, 3.5 in
  private static final double TOWER_UPRIGHT_HEIGHT = 1.831; // m, 72.1 in
  private static final double TOWER_UPRIGHT_SPACING = 0.819; // m, 32.25 in center-to-center

  // Tower rung geometry (Schedule 40 pipe)
  private static final double RUNG_OUTER_DIAMETER = 0.042; // m, 1.66 in OD
  private static final double RUNG_RADIUS = RUNG_OUTER_DIAMETER / 2.0;
  private static final double RUNG_OVERHANG = 0.149; // m, 5.875 in past upright
  private static final double RUNG_LOW_HEIGHT = 0.686; // m, 27 in
  private static final double RUNG_MID_HEIGHT = 1.143; // m, 45 in
  private static final double RUNG_HIGH_HEIGHT = 1.600; // m, 63 in

  // Tower bracing
  private static final double BRACING_BOTTOM = 0.721; // m, 28.4 in
  private static final double BRACING_TOP = 1.102; // m, 43.4 in

  // Outpost geometry
  // Outpost/corral AABBs removed (not in YAGSL collision model, caused phantom floating)

  // Corral geometry

  // Hub ramp area
  private static final double HUB_RAMP_WIDTH = 1.194; // m, 47 in
  private static final double HUB_RAMP_LENGTH = 5.512; // m, 217 in
  private static final double HUB_RAMP_HEIGHT = 0.165; // m, 6.5 in

  // Divider pipe

  // Spatial hash
  private static final double CELL_SIZE = 0.25;
  private static final int GRID_COLS = (int) Math.ceil(FIELD_LENGTH / CELL_SIZE);
  private static final int GRID_ROWS = (int) Math.ceil(FIELD_WIDTH / CELL_SIZE);

  // Max balls
  private static final int MAX_BALLS = 2000;

  // Bump ramp segments (XZ line segments extruded along Y)

  private static final BumpSegment[] BUMP_SEGMENTS = {
    // Blue lower bump ramp up/down
    new BumpSegment(3.96, 0, 4.524, BUMP_HEIGHT, 1.88, 3.73),
    new BumpSegment(4.524, BUMP_HEIGHT, 5.088, 0, 1.88, 3.73),
    // Blue upper bump ramp up/down
    new BumpSegment(3.96, 0, 4.524, BUMP_HEIGHT, 4.32, 6.17),
    new BumpSegment(4.524, BUMP_HEIGHT, 5.088, 0, 4.32, 6.17),
    // Red lower bump ramp up/down
    new BumpSegment(FIELD_LENGTH - 5.088, 0, FIELD_LENGTH - 4.524, BUMP_HEIGHT, 1.88, 3.73),
    new BumpSegment(FIELD_LENGTH - 4.524, BUMP_HEIGHT, FIELD_LENGTH - 3.96, 0, 1.88, 3.73),
    // Red upper bump ramp up/down
    new BumpSegment(FIELD_LENGTH - 5.088, 0, FIELD_LENGTH - 4.524, BUMP_HEIGHT, 4.32, 6.17),
    new BumpSegment(FIELD_LENGTH - 4.524, BUMP_HEIGHT, FIELD_LENGTH - 3.96, 0, 4.32, 6.17),
  };

  // AABB obstacles

  private static final AABB[] AABB_OBSTACLES;

  static {
    List<AABB> aabbs = new ArrayList<>();

    // Trench pillars (4): 12in wide x 53in tall, steel structure
    aabbs.add(
        new AABB(
            3.96,
            TRENCH_WIDTH,
            0,
            3.96 + TRENCH_BLOCK_WIDTH,
            TRENCH_WIDTH + TRENCH_BLOCK_WIDTH,
            TRENCH_PILLAR_HEIGHT,
            COR_STEEL));
    aabbs.add(
        new AABB(
            3.96,
            FIELD_WIDTH - 1.57 - TRENCH_BLOCK_WIDTH,
            0,
            3.96 + TRENCH_BLOCK_WIDTH,
            FIELD_WIDTH - 1.57,
            TRENCH_PILLAR_HEIGHT,
            COR_STEEL));
    aabbs.add(
        new AABB(
            FIELD_LENGTH - 3.96 - TRENCH_BLOCK_WIDTH,
            TRENCH_WIDTH,
            0,
            FIELD_LENGTH - 3.96,
            TRENCH_WIDTH + TRENCH_BLOCK_WIDTH,
            TRENCH_PILLAR_HEIGHT,
            COR_STEEL));
    aabbs.add(
        new AABB(
            FIELD_LENGTH - 3.96 - TRENCH_BLOCK_WIDTH,
            FIELD_WIDTH - 1.57 - TRENCH_BLOCK_WIDTH,
            0,
            FIELD_LENGTH - 3.96,
            FIELD_WIDTH - 1.57,
            TRENCH_PILLAR_HEIGHT,
            COR_STEEL));

    // Trench ceilings (4): steel/aluminum above underpass height
    double trenchXBlue1 = 3.96;
    double trenchXBlue2 = 5.18;
    double trenchXRed1 = FIELD_LENGTH - 5.18;
    double trenchXRed2 = FIELD_LENGTH - 3.96;
    aabbs.add(
        new AABB(
            trenchXBlue1,
            1.57,
            TRENCH_HEIGHT,
            trenchXBlue2,
            3.73,
            TRENCH_PILLAR_HEIGHT,
            COR_STEEL));
    aabbs.add(
        new AABB(
            trenchXBlue1,
            FIELD_WIDTH - 3.73,
            TRENCH_HEIGHT,
            trenchXBlue2,
            FIELD_WIDTH - 1.57,
            TRENCH_PILLAR_HEIGHT,
            COR_STEEL));
    aabbs.add(
        new AABB(
            trenchXRed1, 1.57, TRENCH_HEIGHT, trenchXRed2, 3.73, TRENCH_PILLAR_HEIGHT, COR_STEEL));
    aabbs.add(
        new AABB(
            trenchXRed1,
            FIELD_WIDTH - 3.73,
            TRENCH_HEIGHT,
            trenchXRed2,
            FIELD_WIDTH - 1.57,
            TRENCH_PILLAR_HEIGHT,
            COR_STEEL));

    // Tower poles (2): 2in wide x 47in tall
    double blueTowerX = 1.067;
    double redTowerX = 15.494;
    double blueTowerY = 4.039;
    double redTowerY = 4.318;
    aabbs.add(
        new AABB(
            blueTowerX - TOWER_POLE_WIDTH / 2,
            blueTowerY - TOWER_POLE_WIDTH / 2,
            0,
            blueTowerX + TOWER_POLE_WIDTH / 2,
            blueTowerY + TOWER_POLE_WIDTH / 2,
            TOWER_POLE_HEIGHT,
            COR_STEEL));
    aabbs.add(
        new AABB(
            redTowerX - TOWER_POLE_WIDTH / 2,
            redTowerY - TOWER_POLE_WIDTH / 2,
            0,
            redTowerX + TOWER_POLE_WIDTH / 2,
            redTowerY + TOWER_POLE_WIDTH / 2,
            TOWER_POLE_HEIGHT,
            COR_STEEL));

    // Tower uprights (4): 1.5in x 3.5in x 72.1in, two per tower
    double halfSpacing = TOWER_UPRIGHT_SPACING / 2.0;
    aabbs.add(
        new AABB(
            0,
            blueTowerY - halfSpacing - TOWER_UPRIGHT_THICK / 2,
            0,
            TOWER_UPRIGHT_DEPTH,
            blueTowerY - halfSpacing + TOWER_UPRIGHT_THICK / 2,
            TOWER_UPRIGHT_HEIGHT,
            COR_STEEL));
    aabbs.add(
        new AABB(
            0,
            blueTowerY + halfSpacing - TOWER_UPRIGHT_THICK / 2,
            0,
            TOWER_UPRIGHT_DEPTH,
            blueTowerY + halfSpacing + TOWER_UPRIGHT_THICK / 2,
            TOWER_UPRIGHT_HEIGHT,
            COR_STEEL));
    aabbs.add(
        new AABB(
            FIELD_LENGTH - TOWER_UPRIGHT_DEPTH,
            redTowerY - halfSpacing - TOWER_UPRIGHT_THICK / 2,
            0,
            FIELD_LENGTH,
            redTowerY - halfSpacing + TOWER_UPRIGHT_THICK / 2,
            TOWER_UPRIGHT_HEIGHT,
            COR_STEEL));
    aabbs.add(
        new AABB(
            FIELD_LENGTH - TOWER_UPRIGHT_DEPTH,
            redTowerY + halfSpacing - TOWER_UPRIGHT_THICK / 2,
            0,
            FIELD_LENGTH,
            redTowerY + halfSpacing + TOWER_UPRIGHT_THICK / 2,
            TOWER_UPRIGHT_HEIGHT,
            COR_STEEL));

    // Tower bracing (2): between uprights, 28.4-43.4in height
    aabbs.add(
        new AABB(
            0,
            blueTowerY - halfSpacing,
            BRACING_BOTTOM,
            TOWER_UPRIGHT_DEPTH,
            blueTowerY + halfSpacing,
            BRACING_TOP,
            COR_STEEL));
    aabbs.add(
        new AABB(
            FIELD_LENGTH - TOWER_UPRIGHT_DEPTH,
            redTowerY - halfSpacing,
            BRACING_BOTTOM,
            FIELD_LENGTH,
            redTowerY + halfSpacing,
            BRACING_TOP,
            COR_STEEL));

    // Hub ramp colliders (2): 47in x 217in ground-level area around each hub
    aabbs.add(
        new AABB(
            BLUE_HUB_CENTER.getX() - HUB_RAMP_WIDTH / 2,
            BLUE_HUB_CENTER.getY() - HUB_RAMP_LENGTH / 2,
            0,
            BLUE_HUB_CENTER.getX() + HUB_RAMP_WIDTH / 2,
            BLUE_HUB_CENTER.getY() + HUB_RAMP_LENGTH / 2,
            HUB_RAMP_HEIGHT,
            COR_STEEL));
    aabbs.add(
        new AABB(
            RED_HUB_CENTER.getX() - HUB_RAMP_WIDTH / 2,
            RED_HUB_CENTER.getY() - HUB_RAMP_LENGTH / 2,
            0,
            RED_HUB_CENTER.getX() + HUB_RAMP_WIDTH / 2,
            RED_HUB_CENTER.getY() + HUB_RAMP_LENGTH / 2,
            HUB_RAMP_HEIGHT,
            COR_STEEL));

    // Hub body panels deflect ground-level balls
    aabbs.add(
        new AABB(
            BLUE_HUB_CENTER.getX() - HUB_SIDE / 2,
            BLUE_HUB_CENTER.getY() - HUB_SIDE / 2,
            0,
            BLUE_HUB_CENTER.getX() + HUB_SIDE / 2,
            BLUE_HUB_CENTER.getY() + HUB_SIDE / 2,
            HUB_BASE_HEIGHT,
            COR_HUB));
    aabbs.add(
        new AABB(
            RED_HUB_CENTER.getX() - HUB_SIDE / 2,
            RED_HUB_CENTER.getY() - HUB_SIDE / 2,
            0,
            RED_HUB_CENTER.getX() + HUB_SIDE / 2,
            RED_HUB_CENTER.getY() + HUB_SIDE / 2,
            HUB_BASE_HEIGHT,
            COR_HUB));

    AABB_OBSTACLES = aabbs.toArray(new AABB[0]);
  }

  // Cylinder obstacles (tower rungs + divider pipes)

  private static final CylinderObstacle[] CYLINDER_OBSTACLES;

  static {
    List<CylinderObstacle> cyls = new ArrayList<>();
    double blueTowerY = 4.039;
    double redTowerY = 4.318;
    double halfSpacing = TOWER_UPRIGHT_SPACING / 2.0;

    // Blue tower rungs (3)
    for (double h : new double[] {RUNG_LOW_HEIGHT, RUNG_MID_HEIGHT, RUNG_HIGH_HEIGHT}) {
      cyls.add(
          new CylinderObstacle(
              0,
              blueTowerY - halfSpacing - RUNG_OVERHANG,
              h,
              0,
              blueTowerY + halfSpacing + RUNG_OVERHANG,
              h,
              RUNG_RADIUS,
              COR_STEEL));
    }

    // Red tower rungs (3)
    for (double h : new double[] {RUNG_LOW_HEIGHT, RUNG_MID_HEIGHT, RUNG_HIGH_HEIGHT}) {
      cyls.add(
          new CylinderObstacle(
              FIELD_LENGTH,
              redTowerY - halfSpacing - RUNG_OVERHANG,
              h,
              FIELD_LENGTH,
              redTowerY + halfSpacing + RUNG_OVERHANG,
              h,
              RUNG_RADIUS,
              COR_STEEL));
    }

    CYLINDER_OBSTACLES = cyls.toArray(new CylinderObstacle[0]);
  }

  /** Axis-aligned bounding box with restitution coefficient. */
  private record AABB(
      double minX, double minY, double minZ, double maxX, double maxY, double maxZ, double cor) {}

  /** Cylinder obstacle defined by two axis endpoints, a radius, and restitution. */
  private record CylinderObstacle(
      double ax,
      double ay,
      double az,
      double bx,
      double by,
      double bz,
      double radius,
      double cor,
      double abx,
      double aby,
      double abz,
      double abLenSq) {
    CylinderObstacle(
        double ax,
        double ay,
        double az,
        double bx,
        double by,
        double bz,
        double radius,
        double cor) {
      this(
          ax,
          ay,
          az,
          bx,
          by,
          bz,
          radius,
          cor,
          bx - ax,
          by - ay,
          bz - az,
          (bx - ax) * (bx - ax) + (by - ay) * (by - ay) + (bz - az) * (bz - az));
    }
  }

  /** XZ line segment extruded along a Y range, used for bump ramp geometry. */
  private record BumpSegment(
      double xStart,
      double zStart,
      double xEnd,
      double zEnd,
      double yStart,
      double yEnd,
      double lineX,
      double lineZ,
      double lineLen,
      double nx,
      double nz) {
    BumpSegment(
        double xStart, double zStart, double xEnd, double zEnd, double yStart, double yEnd) {
      this(
          xStart,
          zStart,
          xEnd,
          zEnd,
          yStart,
          yEnd,
          xEnd - xStart,
          zEnd - zStart,
          Math.hypot(xEnd - xStart, zEnd - zStart),
          // Normal perpendicular to line in XZ, flipped so nz >= 0 (points away from ground)
          (xEnd - xStart) >= 0
              ? -(zEnd - zStart) / Math.hypot(xEnd - xStart, zEnd - zStart)
              : (zEnd - zStart) / Math.hypot(xEnd - xStart, zEnd - zStart),
          Math.abs(xEnd - xStart) / Math.hypot(xEnd - xStart, zEnd - zStart));
    }
  }

  /** Physics feature toggles. Flip these on/off to debug or simplify the sim. */
  public static class PhysicsConfig {
    public boolean dragEnabled = true;
    public boolean magnusEnabled = true;
    public boolean frictionEnabled = true;
    public boolean spinTransferEnabled = true;
    public boolean sleepingEnabled = true;
    public boolean ccdEnabled = true;
    public boolean velocityDependentCOR = true;
    public boolean spinDecayEnabled = true;
    public int solverIterations = 4;
    public int subticks = 5;
    public double spinDecayTau = 3.0; // seconds
    public double sleepVelocityThreshold = 0.01; // m/s
    public int sleepFrameThreshold = 10; // consecutive frames
    public double ccdSpeedThreshold = 10.0; // m/s
    public double baumgarteBeta = 0.2;
    public double baumgarteSlop = 0.005; // 5mm allowed penetration
    public boolean deterministic = false;
    public long deterministicSeed = 42L;
    public boolean conservationMonitor = false;

    /** Default: everything on. */
    public PhysicsConfig() {}

    /** Deep copy. */
    public PhysicsConfig copy() {
      PhysicsConfig c = new PhysicsConfig();
      c.dragEnabled = dragEnabled;
      c.magnusEnabled = magnusEnabled;
      c.frictionEnabled = frictionEnabled;
      c.spinTransferEnabled = spinTransferEnabled;
      c.sleepingEnabled = sleepingEnabled;
      c.ccdEnabled = ccdEnabled;
      c.velocityDependentCOR = velocityDependentCOR;
      c.spinDecayEnabled = spinDecayEnabled;
      c.solverIterations = solverIterations;
      c.subticks = subticks;
      c.spinDecayTau = spinDecayTau;
      c.sleepVelocityThreshold = sleepVelocityThreshold;
      c.sleepFrameThreshold = sleepFrameThreshold;
      c.ccdSpeedThreshold = ccdSpeedThreshold;
      c.baumgarteBeta = baumgarteBeta;
      c.baumgarteSlop = baumgarteSlop;
      c.deterministic = deterministic;
      c.deterministicSeed = deterministicSeed;
      c.conservationMonitor = conservationMonitor;
      return c;
    }
  }

  /** One ball in the simulation. Tracks position, velocity, spin, and lifecycle flags. */
  public static class SimBall {
    // State
    Translation3d pos; // field-frame position (m)
    Translation3d vel; // field-frame velocity (m/s)
    Translation3d omega; // angular velocity (rad/s), 3D spin axis

    // Previous state (for CCD and scoring detection)
    Translation3d prevPos;
    Translation3d prevVel;

    // Sleeping
    boolean sleeping;
    int sleepCounter;

    // Stuck-on-obstacle detection
    int elevatedSlowCounter;

    // Lifecycle flags
    boolean intaked;
    boolean outOfBounds;

    SimBall(Translation3d pos, Translation3d vel, Translation3d omega) {
      this.pos = pos;
      this.vel = vel;
      this.omega = omega;
      this.prevPos = pos;
      this.prevVel = vel;
      this.sleeping = false;
      this.sleepCounter = 0;
      this.elevatedSlowCounter = 0;
      this.intaked = false;
      this.outOfBounds = false;
    }

    SimBall(Translation3d pos, Translation3d vel) {
      this(pos, vel, new Translation3d());
    }

    SimBall(Translation3d pos) {
      this(pos, new Translation3d(), new Translation3d());
    }

    /** Get backspin in RPM (from the Y component of omega). */
    public double getSpinRPM() {
      return omega.getY() * 60.0 / (2.0 * Math.PI);
    }
  }

  /** Contact point between two colliding objects. Used by the impulse solver. */
  static class Contact {
    int ballIndexA; // index into balls list
    int ballIndexB; // index into balls list, or -1 for field geometry
    Translation3d normal; // contact normal (A -> B or outward from field)
    double penetration; // overlap depth (positive = overlapping)
    Translation3d contactPoint; // world-space contact location
    double normalImpulseAccum; // warm-start accumulated normal impulse
    double tangentImpulseAccum; // warm-start accumulated tangent impulse
    double restitution; // effective COR for this pair
    double friction; // Coulomb mu for this pair
    double restitutionVelocity; // target bounce-back speed, set once before solving

    Contact() {
      normal = new Translation3d();
      contactPoint = new Translation3d();
    }
  }

  /** A hub that can be scored in. Detects balls falling through the opening. */
  public static class ScoringTarget {
    final Translation2d center;
    final Translation3d exit;
    final int exitVelXSign; // +1 for blue (exits toward red), -1 for red
    int score;

    ScoringTarget(Translation2d center, Translation3d exit, int exitVelXSign) {
      this.center = center;
      this.exit = exit;
      this.exitVelXSign = exitVelXSign;
      this.score = 0;
    }

    boolean didScore(SimBall ball) {
      double dist2d = ball.pos.toTranslation2d().getDistance(center);
      if (dist2d > HUB_ENTRY_RADIUS) return false;
      double currZ = ball.pos.getZ();
      double prevZ = ball.prevPos.getZ();
      // Only count balls falling through the opening (top-down entry)
      return prevZ > HUB_ENTRY_HEIGHT && currZ <= HUB_ENTRY_HEIGHT;
    }

    Translation3d getDispersalVelocity(Random rng) {
      double vx = exitVelXSign * (rng.nextDouble() + 0.1) * 1.5;
      double vy = rng.nextDouble() * 2.0 - 1.0;
      return new Translation3d(vx, vy, 0);
    }

    /** How many balls have scored in this hub. */
    public int getScore() {
      return score;
    }

    void resetScore() {
      score = 0;
    }
  }

  /** Intake zone defined in robot-relative coordinates. Picks up balls that enter the box. */
  static class IntakeZone {
    final double xMin, xMax, yMin, yMax;
    final BooleanSupplier active;
    final Runnable callback;

    IntakeZone(
        double xMin,
        double xMax,
        double yMin,
        double yMax,
        BooleanSupplier active,
        Runnable callback) {
      this.xMin = xMin;
      this.xMax = xMax;
      this.yMin = yMin;
      this.yMax = yMax;
      this.active = active;
      this.callback = callback;
    }

    boolean shouldIntake(SimBall ball, Pose2d robotPose, double bumperHeight) {
      if (!active.getAsBoolean() || ball.pos.getZ() > bumperHeight) return false;
      Translation2d relPos =
          new Pose2d(ball.pos.toTranslation2d(), Rotation2d.kZero)
              .relativeTo(robotPose)
              .getTranslation();
      boolean inside =
          relPos.getX() >= xMin
              && relPos.getX() <= xMax
              && relPos.getY() >= yMin
              && relPos.getY() <= yMax;
      if (inside) {
        callback.run();
      }
      return inside;
    }
  }

  // State

  private final List<SimBall> balls = new ArrayList<>();
  private final List<Contact> contacts = new ArrayList<>();
  private final List<Contact> contactPool = new ArrayList<>(); // pre-allocated contact pool
  private int contactPoolIndex = 0;

  private PhysicsConfig config;
  private Random rng;

  // Spatial hash grid for ball-ball broadphase
  @SuppressWarnings("unchecked")
  private final List<Integer>[][] grid = new ArrayList[GRID_COLS][GRID_ROWS];

  // Hub targets
  private final ScoringTarget blueHub;
  private final ScoringTarget redHub;

  // Robot registration
  private Supplier<Pose2d> robotPoseSupplier;
  private Supplier<ChassisSpeeds> robotSpeedsSupplier;
  private double robotWidth;
  private double robotLength;
  private double bumperHeight;

  // Intakes
  private final List<IntakeZone> intakes = new ArrayList<>();

  // Counters
  private int totalLaunched;
  private int totalScored;
  private int totalIntaked;
  private double lastLaunchSpeed;

  // Running state
  private boolean running;

  // NetworkTables publishing
  private StructArrayPublisher<Translation3d> positionPublisher;
  private StructArrayPublisher<Translation3d> inFlightPublisher;
  private StructArrayPublisher<Translation3d> lastShotArcPublisher;
  private IntegerPublisher blueScorePub;
  private IntegerPublisher redScorePub;
  private IntegerPublisher ballCountPub;
  private IntegerPublisher activeBallsPub;
  private IntegerPublisher sleepingBallsPub;
  private IntegerPublisher contactCountPub;
  private DoublePublisher physicsTimePub;
  private DoublePublisher totalEnergyPub;

  // Last shot arc for trajectory visualization (predicted path in Field3d)
  private Translation3d[] lastShotArc = new Translation3d[0];
  private long lastPhysicsNanos;

  // Conservation monitor state
  private double totalKE;
  private double totalPE;
  private Translation3d totalMomentum = new Translation3d();

  // Constructor

  /**
   * New sim with default physics config. Publishes ball positions to the given NT path.
   *
   * @param tableKey where to publish in NetworkTables (e.g. "Sim/Fuel")
   */
  public FuelPhysicsSim(String tableKey) {
    this(tableKey, new PhysicsConfig());
  }

  /**
   * New sim with custom physics config.
   *
   * @param tableKey where to publish in NetworkTables
   * @param config physics feature toggles and tuning
   */
  public FuelPhysicsSim(String tableKey, PhysicsConfig config) {
    this.config = config;
    this.rng = config.deterministic ? new Random(config.deterministicSeed) : new Random();

    // Initialize spatial hash grid
    for (int i = 0; i < GRID_COLS; i++) {
      for (int j = 0; j < GRID_ROWS; j++) {
        grid[i][j] = new ArrayList<>();
      }
    }

    // Pre-allocate contact pool
    for (int i = 0; i < 200; i++) {
      contactPool.add(new Contact());
    }

    // Create hubs
    blueHub =
        new ScoringTarget(BLUE_HUB_CENTER, new Translation3d(5.3, FIELD_WIDTH / 2.0, 0.89), 1);
    redHub =
        new ScoringTarget(
            RED_HUB_CENTER, new Translation3d(FIELD_LENGTH - 5.3, FIELD_WIDTH / 2.0, 0.89), -1);

    // NT publishers
    var nt = NetworkTableInstance.getDefault();
    positionPublisher =
        nt.getStructArrayTopic(tableKey + "/Positions", Translation3d.struct).publish();
    inFlightPublisher =
        nt.getStructArrayTopic(tableKey + "/InFlight", Translation3d.struct).publish();
    lastShotArcPublisher =
        nt.getStructArrayTopic(tableKey + "/LastShotArc", Translation3d.struct).publish();
    blueScorePub = nt.getIntegerTopic(tableKey + "/BlueScore").publish();
    redScorePub = nt.getIntegerTopic(tableKey + "/RedScore").publish();
    ballCountPub = nt.getIntegerTopic(tableKey + "/Stats/BallCount").publish();
    activeBallsPub = nt.getIntegerTopic(tableKey + "/Stats/ActiveBalls").publish();
    sleepingBallsPub = nt.getIntegerTopic(tableKey + "/Stats/SleepingBalls").publish();
    contactCountPub = nt.getIntegerTopic(tableKey + "/Stats/ContactsPerTick").publish();
    physicsTimePub = nt.getDoubleTopic(tableKey + "/Stats/PhysicsMs").publish();
    totalEnergyPub = nt.getDoubleTopic(tableKey + "/Stats/TotalEnergy").publish();

    running = false;
    totalLaunched = 0;
    totalScored = 0;
    totalIntaked = 0;
    lastLaunchSpeed = 0;
  }

  /** Default constructor, publishes to "Sim/FuelPositions". */
  public FuelPhysicsSim() {
    this("Sim/FuelPositions");
  }

  /** Turn the sim on. You still need to call tick() every cycle. */
  public void enable() {
    running = true;
  }

  /** Pause the sim. Balls freeze in place. */
  public void disable() {
    running = false;
  }

  /** Is the sim running? */
  public boolean isRunning() {
    return running;
  }

  /**
   * Tell the sim about your robot so it can handle bumper collisions and intake pickup.
   *
   * @param width robot width along Y axis (m)
   * @param length robot length along X axis (m)
   * @param bumperHeight bumper height (m)
   * @param poseSupplier field-relative pose supplier
   * @param speedsSupplier field-relative chassis speeds supplier
   */
  public void configureRobot(
      double width,
      double length,
      double bumperHeight,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> speedsSupplier) {
    this.robotWidth = width;
    this.robotLength = length;
    this.bumperHeight = bumperHeight;
    this.robotPoseSupplier = poseSupplier;
    this.robotSpeedsSupplier = speedsSupplier;
  }

  /**
   * Add an intake zone. Balls that enter this box (in robot-relative coords) get picked up.
   *
   * @param xMin front edge in robot frame
   * @param xMax back edge in robot frame
   * @param yMin left edge in robot frame
   * @param yMax right edge in robot frame
   * @param active returns true when the intake is actually running
   * @param callback fires when a ball gets picked up
   */
  public void addIntakeZone(
      double xMin,
      double xMax,
      double yMin,
      double yMax,
      BooleanSupplier active,
      Runnable callback) {
    intakes.add(new IntakeZone(xMin, xMax, yMin, yMax, active, callback));
  }

  /** Add an intake zone without a callback. */
  public void addIntakeZone(
      double xMin, double xMax, double yMin, double yMax, BooleanSupplier active) {
    addIntakeZone(xMin, xMax, yMin, yMax, active, () -> {});
  }

  /**
   * Shoot a ball into the sim.
   *
   * @param pos where the ball leaves the launcher (field frame, meters)
   * @param vel launch velocity (field frame, m/s)
   * @param spinRPM backspin in RPM (positive = backspin = Magnus lift)
   */
  public void launchBall(Translation3d pos, Translation3d vel, double spinRPM) {
    // Convert RPM to 3D omega: backspin is rotation around -Y axis
    double omegaY = -spinRPM * 2.0 * Math.PI / 60.0;
    Translation3d omega = new Translation3d(0, omegaY, 0);
    launchBall(pos, vel, omega);
  }

  /**
   * Shoot a ball with full 3D spin control.
   *
   * @param pos launch position (field frame, meters)
   * @param vel launch velocity (field frame, m/s)
   * @param omega 3D angular velocity (rad/s)
   */
  public void launchBall(Translation3d pos, Translation3d vel, Translation3d omega) {
    if (balls.size() >= MAX_BALLS) return;
    SimBall ball = new SimBall(pos, vel, omega);
    balls.add(ball);
    totalLaunched++;
    lastLaunchSpeed = vel.getNorm();

    // Predict the trajectory arc for Field3d visualization (20 points, gravity + drag only)
    lastShotArc = predictArc(pos, vel, 20, 0.05);

    // Wake nearby sleeping balls
    if (config.sleepingEnabled) {
      wakeNearbyBalls(pos, 1.0);
    }
  }

  /** Drop a ball at this position, sitting on the ground. */
  public void spawnBall(Translation3d pos) {
    if (balls.size() >= MAX_BALLS) return;
    balls.add(new SimBall(pos));
  }

  /** Drop a ball with some initial velocity. */
  public void spawnBall(Translation3d pos, Translation3d vel) {
    if (balls.size() >= MAX_BALLS) return;
    balls.add(new SimBall(pos, vel));
  }

  /** Remove every ball from the sim. */
  public void clearBalls() {
    balls.clear();
  }

  /** Spawn all game pieces in their starting positions (neutral zone + depots). */
  public void placeFieldBalls() {
    // Neutral zone fuel
    double cx = FIELD_LENGTH / 2.0;
    double cy = FIELD_WIDTH / 2.0;
    int nzCols = 12; // 12 * 5.91in = 70.9in, fits inside the 72.0in depth
    int nzRows = 30; // 30 * 5.91in = 177.3in, fits inside the 206.0in width
    double halfDivider = 0.0254; // half of the 2.0in center divider

    for (int col = 0; col < nzCols; col++) {
      double x = cx + (col - (nzCols - 1) * 0.5) * BALL_DIAMETER;
      for (int row = 0; row < nzRows; row++) {
        double y = cy + (row - (nzRows - 1) * 0.5) * BALL_DIAMETER;
        if (Math.abs(y - cy) < halfDivider) continue;
        spawnBall(new Translation3d(x, y, BALL_RADIUS));
      }
    }

    // Depot fuel
    int depotCols = 4; // 4 * 5.91in = 23.6in fits inside 27.0in depth
    int depotRows = 6; // 6 * 5.91in = 35.5in fits inside 42.0in width

    // Blue-side depot
    fillBallGrid(FIELD_LENGTH - 0.37, 2.10, depotCols, depotRows);
    // Red-side depot
    fillBallGrid(0.37, FIELD_WIDTH - 2.10, depotCols, depotRows);
  }

  /** Helper: fill a grid of balls centered at (cx, cy). */
  private void fillBallGrid(double cx, double cy, int cols, int rows) {
    for (int c = 0; c < cols; c++) {
      double x = cx + (c - (cols - 1) * 0.5) * BALL_DIAMETER;
      for (int r = 0; r < rows; r++) {
        double y = cy + (r - (rows - 1) * 0.5) * BALL_DIAMETER;
        spawnBall(new Translation3d(x, y, BALL_RADIUS));
      }
    }
  }

  /**
   * Step the sim forward one period (20ms) and publish ball positions to NT. Does nothing if the
   * sim isn't enabled.
   */
  public void tick() {
    if (!running) return;
    long t0 = System.nanoTime();
    advancePhysics(PERIOD);
    lastPhysicsNanos = System.nanoTime() - t0;
    publishPositions();
  }

  /**
   * Advance physics by dt seconds. Splits into subticks for stability.
   *
   * @param dt time to advance (seconds)
   */
  public void advancePhysics(double dt) {
    int ticks = Math.max(1, config.subticks);
    double subDt = dt / ticks;

    for (int tick = 0; tick < ticks; tick++) {
      stepSubtick(subDt);
    }

    // Remove flagged balls
    removeFlaggedBalls();
  }

  // Core physics pipeline

  private void stepSubtick(double subDt) {
    // Reset contact list
    contactPoolIndex = 0;
    contacts.clear();

    for (int i = 0; i < balls.size(); i++) {
      SimBall ball = balls.get(i);
      if (ball.intaked || ball.outOfBounds) continue;
      if (ball.sleeping && config.sleepingEnabled) continue;

      // Save previous state
      ball.prevPos = ball.pos;
      ball.prevVel = ball.vel;

      // Compute forces and get acceleration
      Translation3d accel = computeAcceleration(ball);

      // Symplectic Euler: update velocity first so we don't accumulate energy drift
      ball.vel = ball.vel.plus(accel.times(subDt));
      ball.pos = ball.pos.plus(ball.vel.times(subDt));

      // Spin decay
      if (config.spinDecayEnabled && ball.omega.getNorm() > 1e-6) {
        double decayFactor = Math.exp(-subDt / config.spinDecayTau);
        ball.omega = ball.omega.times(decayFactor);
      }
    }

    // CCD for fast balls
    if (config.ccdEnabled) {
      for (int i = 0; i < balls.size(); i++) {
        SimBall ball = balls.get(i);
        if (ball.intaked || ball.outOfBounds) continue;
        if (ball.sleeping && config.sleepingEnabled) continue;
        if (ball.vel.getNorm() > config.ccdSpeedThreshold) {
          handleCCD(ball);
        }
      }
    }

    // Broadphase: build spatial hash
    buildSpatialHash();

    // Generate contacts: ball-ball via spatial hash, ball-field via narrowphase
    generateBallBallContacts();
    generateBallFieldContacts();

    // Sequential impulse solver
    solveContacts();

    // Simple wall/ground handling (direct impulse, not through solver)
    for (int i = 0; i < balls.size(); i++) {
      SimBall ball = balls.get(i);
      if (ball.intaked || ball.outOfBounds) continue;
      if (ball.sleeping && config.sleepingEnabled) continue;
      handleWallBounce(ball);
      handleGroundContact(ball, subDt);
      handleBumpCollisions(ball);
    }

    // Hub scoring
    for (int i = 0; i < balls.size(); i++) {
      SimBall ball = balls.get(i);
      if (ball.intaked || ball.outOfBounds) continue;
      handleHubScoring(ball);
    }

    // Net collisions
    for (int i = 0; i < balls.size(); i++) {
      SimBall ball = balls.get(i);
      if (ball.intaked || ball.outOfBounds) continue;
      if (ball.sleeping && config.sleepingEnabled) continue;
      handleNetCollision(ball, blueHub);
      handleNetCollision(ball, redHub);
    }

    // Robot interaction
    if (robotPoseSupplier != null && robotSpeedsSupplier != null) {
      Pose2d robotPose = robotPoseSupplier.get();
      ChassisSpeeds speeds = robotSpeedsSupplier.get();
      Translation2d robotVel =
          new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

      // Wake radius: robot half-diagonal plus margin so balls react before contact
      double wakeRadius = Math.hypot(robotLength, robotWidth) / 2.0 + 0.3;
      double wakeRadiusSq = wakeRadius * wakeRadius;

      for (int i = 0; i < balls.size(); i++) {
        SimBall ball = balls.get(i);
        if (ball.intaked || ball.outOfBounds) continue;
        // Wake sleeping balls near the robot so bumpers push them
        if (ball.sleeping && config.sleepingEnabled) {
          double dx = ball.pos.getX() - robotPose.getX();
          double dy = ball.pos.getY() - robotPose.getY();
          if (dx * dx + dy * dy < wakeRadiusSq) {
            wakeBall(ball);
          } else {
            continue;
          }
        }
        // Intake first so balls are consumed before bumper pushes them away
        handleIntakePickup(ball, robotPose);
        if (ball.intaked) continue;
        handleRobotCollision(ball, robotPose, robotVel);
      }
    }

    // Sleep update
    if (config.sleepingEnabled) {
      for (int i = 0; i < balls.size(); i++) {
        updateSleepState(balls.get(i));
      }
    }

    // Out-of-bounds cleanup (includes NaN guard, upper Z limit, and stuck-on-obstacle removal)
    for (int i = 0; i < balls.size(); i++) {
      SimBall ball = balls.get(i);
      if (!Double.isFinite(ball.pos.getX())
          || !Double.isFinite(ball.pos.getY())
          || !Double.isFinite(ball.pos.getZ())
          || !Double.isFinite(ball.vel.getX())
          || !Double.isFinite(ball.vel.getY())
          || !Double.isFinite(ball.vel.getZ())
          || !Double.isFinite(ball.omega.getX())
          || !Double.isFinite(ball.omega.getY())
          || !Double.isFinite(ball.omega.getZ())
          || ball.pos.getX() < -2.0
          || ball.pos.getX() > FIELD_LENGTH + 2.0
          || ball.pos.getY() < -2.0
          || ball.pos.getY() > FIELD_WIDTH + 2.0
          || ball.pos.getZ() < -1.0
          || ball.pos.getZ() > 15.0) {
        ball.outOfBounds = true;
      }
      // Remove balls stuck on elevated obstacles
      if (ball.pos.getZ() > BALL_RADIUS + 0.3 && ball.vel.getNorm() < 0.5) {
        ball.elevatedSlowCounter++;
        if (ball.elevatedSlowCounter > 250) {
          ball.outOfBounds = true;
        }
      } else {
        ball.elevatedSlowCounter = 0;
      }
    }

    // Conservation monitor
    if (config.conservationMonitor) {
      computeConservationQuantities();
    }
  }

  /** Compute acceleration: gravity + drag + Magnus lift. Magnus only kicks in when airborne. */
  private Translation3d computeAcceleration(SimBall ball) {
    // Gravity always acts
    double ax = 0, ay = 0, az = -GRAVITY;

    double speed = ball.vel.getNorm();
    boolean airborne = ball.pos.getZ() > BALL_RADIUS + 0.01;

    // Drag acts whether on ground or airborne (air doesn't vanish at carpet level)
    if (config.dragEnabled && speed > 1e-6) {
      ax -= DRAG_ACCEL_FACTOR * speed * ball.vel.getX();
      ay -= DRAG_ACCEL_FACTOR * speed * ball.vel.getY();
      az -= DRAG_ACCEL_FACTOR * speed * ball.vel.getZ();
    }

    // Magnus only applies airborne (ground friction dominates spin behavior on carpet)
    if (airborne && speed > 1e-6) {
      if (config.magnusEnabled && ball.omega.getNorm() > 1e-3) {
        Translation3d magnusDir = cross(ball.omega, ball.vel);
        double magnusMag = magnusDir.getNorm();
        if (magnusMag > 1e-6) {
          // a_magnus = MAGNUS_ACCEL_FACTOR * (omega x v)
          ax += MAGNUS_ACCEL_FACTOR * magnusDir.getX();
          ay += MAGNUS_ACCEL_FACTOR * magnusDir.getY();
          az += MAGNUS_ACCEL_FACTOR * magnusDir.getZ();
        }
      }
    }

    return new Translation3d(ax, ay, az);
  }

  /** Continuous collision detection: sweep fast balls so they don't tunnel through walls. */
  private void handleCCD(SimBall ball) {
    Translation3d delta = ball.pos.minus(ball.prevPos);
    double dist = delta.getNorm();
    if (dist < 1e-6) return;

    Translation3d dir = delta.div(dist);

    // Check against field boundaries
    double tMin = 1.0;
    Translation3d hitNormal = null;

    // X walls
    if (dir.getX() < -1e-6) {
      double t = (BALL_RADIUS - ball.prevPos.getX()) / (delta.getX());
      if (t > 0 && t < tMin) {
        tMin = t;
        hitNormal = AXIS_X_POS;
      }
    } else if (dir.getX() > 1e-6) {
      double t = (FIELD_LENGTH - BALL_RADIUS - ball.prevPos.getX()) / (delta.getX());
      if (t > 0 && t < tMin) {
        tMin = t;
        hitNormal = AXIS_X_NEG;
      }
    }

    // Y walls
    if (dir.getY() < -1e-6) {
      double t = (BALL_RADIUS - ball.prevPos.getY()) / (delta.getY());
      if (t > 0 && t < tMin) {
        tMin = t;
        hitNormal = AXIS_Y_POS;
      }
    } else if (dir.getY() > 1e-6) {
      double t = (FIELD_WIDTH - BALL_RADIUS - ball.prevPos.getY()) / (delta.getY());
      if (t > 0 && t < tMin) {
        tMin = t;
        hitNormal = AXIS_Y_NEG;
      }
    }

    // Ground
    if (dir.getZ() < -1e-6) {
      double t = (BALL_RADIUS - ball.prevPos.getZ()) / (delta.getZ());
      if (t > 0 && t < tMin) {
        tMin = t;
        hitNormal = AXIS_Z_POS;
      }
    }

    // AABB obstacles (ray-box intersection)
    for (AABB aabb : AABB_OBSTACLES) {
      double tHit = sweepSphereAABB(ball.prevPos, delta, aabb);
      if (tHit >= 0 && tHit < tMin) {
        // Compute normal at hit point
        Translation3d hitPos = ball.prevPos.plus(delta.times(tHit));
        Translation3d n = computeAABBNormal(hitPos, aabb);
        if (n != null) {
          tMin = tHit;
          hitNormal = n;
        }
      }
    }

    if (hitNormal != null && tMin < 1.0) {
      // Move ball to contact point
      ball.pos = ball.prevPos.plus(delta.times(tMin));

      // Reflect velocity
      double vDotN = ball.vel.dot(hitNormal);
      if (vDotN < 0) {
        double cor = config.velocityDependentCOR ? velocityCOR(COR_WALL, -vDotN) : COR_WALL;
        ball.vel = ball.vel.minus(hitNormal.times((1.0 + cor) * vDotN));
      }
    }
  }

  /**
   * Ray-AABB intersection with sphere expansion (Minkowski sum). Returns hit time in [0,1] or -1.
   */
  private double sweepSphereAABB(Translation3d origin, Translation3d delta, AABB aabb) {
    // Expand AABB by ball radius (Minkowski sum with sphere)
    double minX = aabb.minX() - BALL_RADIUS;
    double minY = aabb.minY() - BALL_RADIUS;
    double minZ = aabb.minZ() - BALL_RADIUS;
    double maxX = aabb.maxX() + BALL_RADIUS;
    double maxY = aabb.maxY() + BALL_RADIUS;
    double maxZ = aabb.maxZ() + BALL_RADIUS;

    double tEnter = 0;
    double tExit = 1;

    // X slab
    if (Math.abs(delta.getX()) > 1e-9) {
      double invD = 1.0 / delta.getX();
      double t1 = (minX - origin.getX()) * invD;
      double t2 = (maxX - origin.getX()) * invD;
      if (t1 > t2) {
        double tmp = t1;
        t1 = t2;
        t2 = tmp;
      }
      tEnter = Math.max(tEnter, t1);
      tExit = Math.min(tExit, t2);
    } else {
      if (origin.getX() < minX || origin.getX() > maxX) return -1;
    }

    // Y slab
    if (Math.abs(delta.getY()) > 1e-9) {
      double invD = 1.0 / delta.getY();
      double t1 = (minY - origin.getY()) * invD;
      double t2 = (maxY - origin.getY()) * invD;
      if (t1 > t2) {
        double tmp = t1;
        t1 = t2;
        t2 = tmp;
      }
      tEnter = Math.max(tEnter, t1);
      tExit = Math.min(tExit, t2);
    } else {
      if (origin.getY() < minY || origin.getY() > maxY) return -1;
    }

    // Z slab
    if (Math.abs(delta.getZ()) > 1e-9) {
      double invD = 1.0 / delta.getZ();
      double t1 = (minZ - origin.getZ()) * invD;
      double t2 = (maxZ - origin.getZ()) * invD;
      if (t1 > t2) {
        double tmp = t1;
        t1 = t2;
        t2 = tmp;
      }
      tEnter = Math.max(tEnter, t1);
      tExit = Math.min(tExit, t2);
    } else {
      if (origin.getZ() < minZ || origin.getZ() > maxZ) return -1;
    }

    if (tEnter > tExit || tExit < 0) return -1;
    return tEnter > 0 ? tEnter : -1; // Already inside if tEnter <= 0
  }

  // Broadphase (spatial hash)

  private void buildSpatialHash() {
    for (int i = 0; i < GRID_COLS; i++) {
      for (int j = 0; j < GRID_ROWS; j++) {
        grid[i][j].clear();
      }
    }
    // Include sleeping balls so awake balls can detect them for wake-on-collision
    for (int i = 0; i < balls.size(); i++) {
      SimBall ball = balls.get(i);
      if (ball.intaked || ball.outOfBounds) continue;
      int col = (int) (ball.pos.getX() / CELL_SIZE);
      int row = (int) (ball.pos.getY() / CELL_SIZE);
      if (col >= 0 && col < GRID_COLS && row >= 0 && row < GRID_ROWS) {
        grid[col][row].add(i);
      }
    }
  }

  // Narrowphase contact generation

  private void generateBallBallContacts() {
    for (int i = 0; i < balls.size(); i++) {
      SimBall ballA = balls.get(i);
      if (ballA.intaked || ballA.outOfBounds) continue;
      if (ballA.sleeping && config.sleepingEnabled) continue;

      int col = (int) (ballA.pos.getX() / CELL_SIZE);
      int row = (int) (ballA.pos.getY() / CELL_SIZE);

      for (int di = -1; di <= 1; di++) {
        for (int dj = -1; dj <= 1; dj++) {
          int ci = col + di;
          int cj = row + dj;
          if (ci < 0 || ci >= GRID_COLS || cj < 0 || cj >= GRID_ROWS) continue;
          List<Integer> cell = grid[ci][cj];
          for (int k = 0; k < cell.size(); k++) {
            int j = cell.get(k);
            if (j == i) continue; // same ball
            if (j < i && !(config.sleepingEnabled && balls.get(j).sleeping)) continue;

            SimBall ballB = balls.get(j);
            double dx = ballA.pos.getX() - ballB.pos.getX();
            double dy = ballA.pos.getY() - ballB.pos.getY();
            double dz = ballA.pos.getZ() - ballB.pos.getZ();
            double distSq = dx * dx + dy * dy + dz * dz;
            double minDist = BALL_RADIUS * 2;

            if (distSq < minDist * minDist) {
              double dist = Math.sqrt(distSq);
              Contact c = allocateContact();
              if (dist < 1e-9) {
                c.normal = AXIS_X_POS;
                dist = 1e-9;
              } else {
                c.normal = ballA.pos.minus(ballB.pos).div(dist);
              }
              c.ballIndexA = i;
              c.ballIndexB = j;
              c.penetration = minDist - dist;
              c.contactPoint = ballA.pos.plus(ballB.pos).div(2.0); // midpoint between centers
              c.restitution = COR_BALL_BALL;
              c.friction = config.frictionEnabled ? MU_BALL_BALL : 0;
              c.normalImpulseAccum = 0;
              c.tangentImpulseAccum = 0;
              contacts.add(c);

              // Wake sleeping ball on contact
              if (ballB.sleeping) {
                wakeBall(ballB);
              }
            }
          }
        }
      }
    }
  }

  private void generateBallFieldContacts() {
    for (int i = 0; i < balls.size(); i++) {
      SimBall ball = balls.get(i);
      if (ball.intaked || ball.outOfBounds) continue;
      if (ball.sleeping && config.sleepingEnabled) continue;

      // AABB obstacles
      for (AABB aabb : AABB_OBSTACLES) {
        generateSphereAABBContact(i, ball, aabb);
      }

      // Cylinder obstacles
      for (CylinderObstacle cyl : CYLINDER_OBSTACLES) {
        generateSphereCylinderContact(i, ball, cyl);
      }
    }
  }

  private void generateSphereAABBContact(int ballIndex, SimBall ball, AABB aabb) {
    // Find nearest point on AABB to sphere center
    double cx = Math.max(aabb.minX(), Math.min(ball.pos.getX(), aabb.maxX()));
    double cy = Math.max(aabb.minY(), Math.min(ball.pos.getY(), aabb.maxY()));
    double cz = Math.max(aabb.minZ(), Math.min(ball.pos.getZ(), aabb.maxZ()));

    double dx = ball.pos.getX() - cx;
    double dy = ball.pos.getY() - cy;
    double dz = ball.pos.getZ() - cz;
    double distSq = dx * dx + dy * dy + dz * dz;

    if (distSq < BALL_RADIUS * BALL_RADIUS && distSq >= 1e-9) {
      double dist = Math.sqrt(distSq);
      Contact c = allocateContact();
      c.ballIndexA = ballIndex;
      c.ballIndexB = -1;
      c.normal = new Translation3d(dx / dist, dy / dist, dz / dist);
      c.penetration = BALL_RADIUS - dist;
      c.contactPoint = new Translation3d(cx, cy, cz);
      c.restitution = aabb.cor();
      c.friction = config.frictionEnabled ? MU_WALL : 0;
      c.normalImpulseAccum = 0;
      c.tangentImpulseAccum = 0;
      contacts.add(c);
    } else if (distSq < 1e-9) {
      // Ball center is inside AABB.
      Translation3d normal = computeEntryFaceNormal(ball.prevPos, ball.pos, aabb);
      if (normal != null) {
        double pen = computeAABBPenetration(ball.pos, aabb);
        Contact c = allocateContact();
        c.ballIndexA = ballIndex;
        c.ballIndexB = -1;
        c.normal = normal;
        c.penetration = pen + BALL_RADIUS;
        c.contactPoint = ball.pos;
        c.restitution = aabb.cor();
        c.friction = config.frictionEnabled ? MU_WALL : 0;
        c.normalImpulseAccum = 0;
        c.tangentImpulseAccum = 0;
        contacts.add(c);
      }
    }
  }

  /** Which face of the AABB is the point closest to? Returns outward normal of that face. */
  private Translation3d computeAABBNormal(Translation3d p, AABB aabb) {
    double dxMin = p.getX() - aabb.minX();
    double dxMax = aabb.maxX() - p.getX();
    double dyMin = p.getY() - aabb.minY();
    double dyMax = aabb.maxY() - p.getY();
    double dzMin = p.getZ() - aabb.minZ();
    double dzMax = aabb.maxZ() - p.getZ();

    double min = dxMin;
    Translation3d normal = AXIS_X_NEG;

    if (dxMax < min) {
      min = dxMax;
      normal = AXIS_X_POS;
    }
    if (dyMin < min) {
      min = dyMin;
      normal = AXIS_Y_NEG;
    }
    if (dyMax < min) {
      min = dyMax;
      normal = AXIS_Y_POS;
    }
    if (dzMin < min) {
      min = dzMin;
      normal = AXIS_Z_NEG;
    }
    if (dzMax < min) {
      min = dzMax;
      normal = AXIS_Z_POS;
    }
    return normal;
  }

  /** How deep is a point inside an AABB? Returns min distance to any face. */
  private double computeAABBPenetration(Translation3d p, AABB aabb) {
    double dxMin = p.getX() - aabb.minX();
    double dxMax = aabb.maxX() - p.getX();
    double dyMin = p.getY() - aabb.minY();
    double dyMax = aabb.maxY() - p.getY();
    double dzMin = p.getZ() - aabb.minZ();
    double dzMax = aabb.maxZ() - p.getZ();
    return Math.min(
        Math.min(Math.min(dxMin, dxMax), Math.min(dyMin, dyMax)), Math.min(dzMin, dzMax));
  }

  /**
   * If a ball is inside an AABB, figure out which face it came in through by raycasting from
   * prevPos to pos. Without this, balls pop onto the top of obstacles when they actually entered
   * from the side. Falls back to nearest-face if the ray is degenerate (ball spawned inside).
   */
  private Translation3d computeEntryFaceNormal(Translation3d from, Translation3d to, AABB aabb) {
    if (from == null) return computeAABBNormal(to, aabb);

    double dx = to.getX() - from.getX();
    double dy = to.getY() - from.getY();
    double dz = to.getZ() - from.getZ();

    // Slab intersection: the entry face is the axis with the latest entry time
    double tMax = Double.NEGATIVE_INFINITY;
    Translation3d bestNormal = null;

    if (Math.abs(dx) > 1e-12) {
      double invD = 1.0 / dx;
      double t1 = (aabb.minX() - from.getX()) * invD;
      double t2 = (aabb.maxX() - from.getX()) * invD;
      double tEntry = Math.min(t1, t2);
      if (tEntry > tMax) {
        tMax = tEntry;
        bestNormal = dx > 0 ? AXIS_X_NEG : AXIS_X_POS;
      }
    }
    if (Math.abs(dy) > 1e-12) {
      double invD = 1.0 / dy;
      double t1 = (aabb.minY() - from.getY()) * invD;
      double t2 = (aabb.maxY() - from.getY()) * invD;
      double tEntry = Math.min(t1, t2);
      if (tEntry > tMax) {
        tMax = tEntry;
        bestNormal = dy > 0 ? AXIS_Y_NEG : AXIS_Y_POS;
      }
    }
    if (Math.abs(dz) > 1e-12) {
      double invD = 1.0 / dz;
      double t1 = (aabb.minZ() - from.getZ()) * invD;
      double t2 = (aabb.maxZ() - from.getZ()) * invD;
      double tEntry = Math.min(t1, t2);
      if (tEntry > tMax) {
        tMax = tEntry;
        bestNormal = dz > 0 ? AXIS_Z_NEG : AXIS_Z_POS;
      }
    }

    return bestNormal != null ? bestNormal : computeAABBNormal(to, aabb);
  }

  private void generateSphereCylinderContact(int ballIndex, SimBall ball, CylinderObstacle cyl) {
    if (cyl.abLenSq() < 1e-12) return;

    // Find nearest point on line segment to ball center

    double apx = ball.pos.getX() - cyl.ax();
    double apy = ball.pos.getY() - cyl.ay();
    double apz = ball.pos.getZ() - cyl.az();
    double t =
        Math.max(
            0, Math.min(1, (apx * cyl.abx() + apy * cyl.aby() + apz * cyl.abz()) / cyl.abLenSq()));

    double nearX = cyl.ax() + t * cyl.abx();
    double nearY = cyl.ay() + t * cyl.aby();
    double nearZ = cyl.az() + t * cyl.abz();

    double dx = ball.pos.getX() - nearX;
    double dy = ball.pos.getY() - nearY;
    double dz = ball.pos.getZ() - nearZ;
    double distSq = dx * dx + dy * dy + dz * dz;
    double minDist = BALL_RADIUS + cyl.radius();

    if (distSq < minDist * minDist && distSq > 0) {
      double dist = Math.sqrt(distSq);
      Contact c = allocateContact();
      c.ballIndexA = ballIndex;
      c.ballIndexB = -1;
      c.normal = new Translation3d(dx / dist, dy / dist, dz / dist);
      c.penetration = minDist - dist;
      c.contactPoint = new Translation3d(nearX, nearY, nearZ);
      c.restitution = cyl.cor();
      c.friction = config.frictionEnabled ? MU_WALL : 0;
      c.normalImpulseAccum = 0;
      c.tangentImpulseAccum = 0;
      contacts.add(c);
    }
  }

  /**
   * Sequential impulse solver: resolve bounces, friction, and spin transfer across all contacts.
   */
  private void solveContacts() {
    if (contacts.isEmpty()) return;

    // Compute restitution targets from initial approach velocities (before any solving)
    for (int i = 0; i < contacts.size(); i++) {
      Contact c = contacts.get(i);
      SimBall ballA = balls.get(c.ballIndexA);
      Translation3d relVel;
      if (c.ballIndexB >= 0) {
        relVel = ballA.vel.minus(balls.get(c.ballIndexB).vel);
      } else {
        relVel = ballA.vel;
      }
      double vn = relVel.dot(c.normal);
      double e = c.restitution;
      if (config.velocityDependentCOR) {
        e = velocityCOR(e, Math.abs(vn));
      }
      // Only bounce for significant approach speeds.
      c.restitutionVelocity = vn < -0.5 ? -e * vn : 0;
    }

    for (int iter = 0; iter < config.solverIterations; iter++) {
      for (int i = 0; i < contacts.size(); i++) {
        Contact c = contacts.get(i);
        solveContact(c);
      }
    }

    // Position correction
    for (int i = 0; i < contacts.size(); i++) {
      applyPositionCorrection(contacts.get(i));
    }
  }

  private void solveContact(Contact c) {
    SimBall ballA = balls.get(c.ballIndexA);
    Translation3d relVel;
    double invMassSum;

    if (c.ballIndexB >= 0) {
      // Ball-ball contact
      SimBall ballB = balls.get(c.ballIndexB);
      relVel = ballA.vel.minus(ballB.vel);
      invMassSum = 2.0 / BALL_MASS; // both balls have same mass
    } else {
      // Ball-field contact
      relVel = ballA.vel;
      invMassSum = 1.0 / BALL_MASS;
    }

    double vRelNormal = relVel.dot(c.normal);
    double jn = -(vRelNormal - c.restitutionVelocity) / invMassSum;
    double oldAccum = c.normalImpulseAccum;
    c.normalImpulseAccum = Math.max(0, oldAccum + jn);
    jn = c.normalImpulseAccum - oldAccum;

    // Apply normal impulse
    Translation3d normalImpulse = c.normal.times(jn);
    ballA.vel = ballA.vel.plus(normalImpulse.div(BALL_MASS));
    if (c.ballIndexB >= 0) {
      SimBall ballB = balls.get(c.ballIndexB);
      ballB.vel = ballB.vel.minus(normalImpulse.div(BALL_MASS));
    }

    // Tangential impulse (friction)
    if (c.friction > 0 && config.frictionEnabled) {
      // Recompute relative velocity after normal impulse
      if (c.ballIndexB >= 0) {
        relVel = ballA.vel.minus(balls.get(c.ballIndexB).vel);
      } else {
        relVel = ballA.vel;
      }

      // Tangent velocity: remove normal component
      double vn = relVel.dot(c.normal);
      Translation3d vTangent = relVel.minus(c.normal.times(vn));
      double vTangentMag = vTangent.getNorm();

      if (vTangentMag > 1e-6) {
        Translation3d tangentDir = vTangent.div(vTangentMag);

        // Friction impulse magnitude
        double jt = -vTangentMag / invMassSum;

        // Coulomb clamp: |j_t| <= mu * j_n
        double maxFriction = c.friction * c.normalImpulseAccum;
        double oldTangentAccum = c.tangentImpulseAccum;
        c.tangentImpulseAccum = Math.max(-maxFriction, Math.min(maxFriction, oldTangentAccum + jt));
        jt = c.tangentImpulseAccum - oldTangentAccum;

        // Apply tangent impulse
        Translation3d frictionImpulse = tangentDir.times(jt);
        ballA.vel = ballA.vel.plus(frictionImpulse.div(BALL_MASS));
        if (c.ballIndexB >= 0) {
          SimBall ballB = balls.get(c.ballIndexB);
          ballB.vel = ballB.vel.minus(frictionImpulse.div(BALL_MASS));
        }

        // Spin transfer from friction torque
        if (config.spinTransferEnabled && Math.abs(jt) > 1e-9) {
          Translation3d rContact = c.normal.times(-BALL_RADIUS); // from center to contact
          Translation3d torqueImpulse = cross(rContact, frictionImpulse);
          Translation3d deltaOmega = torqueImpulse.div(BALL_MOMENT_OF_INERTIA);
          ballA.omega = ballA.omega.plus(deltaOmega);

          if (c.ballIndexB >= 0) {
            SimBall ballB = balls.get(c.ballIndexB);
            Translation3d rContactB = c.normal.times(BALL_RADIUS);
            Translation3d torqueImpulseB = cross(rContactB, frictionImpulse.unaryMinus());
            Translation3d deltaOmegaB = torqueImpulseB.div(BALL_MOMENT_OF_INERTIA);
            ballB.omega = ballB.omega.plus(deltaOmegaB);
          }
        }
      }
    }
  }

  /** Push overlapping objects apart so they don't sink into each other. */
  private void applyPositionCorrection(Contact c) {
    double slop = config.baumgarteSlop;
    double beta = config.baumgarteBeta;
    double correction = Math.max(c.penetration - slop, 0) * beta;

    if (correction < 1e-6) return;

    SimBall ballA = balls.get(c.ballIndexA);
    if (c.ballIndexB >= 0) {
      SimBall ballB = balls.get(c.ballIndexB);
      Translation3d corrVec = c.normal.times(correction * 0.5);
      ballA.pos = ballA.pos.plus(corrVec);
      ballB.pos = ballB.pos.minus(corrVec);
    } else {
      ballA.pos = ballA.pos.plus(c.normal.times(correction));
    }
  }

  // Wall and ground handling

  private void handleWallBounce(SimBall ball) {
    double z = ball.pos.getZ();

    // X walls (alliance walls: diamond plate + polycarbonate, height-aware)
    if (ball.pos.getX() < BALL_RADIUS) {
      if (z < ALLIANCE_WALL_HEIGHT) {
        ball.pos = new Translation3d(BALL_RADIUS, ball.pos.getY(), ball.pos.getZ());
        if (ball.vel.getX() < 0) {
          applyWallSpinTransfer(ball, AXIS_X_POS);
          double cor = effectiveCOR(COR_WALL, Math.abs(ball.vel.getX()));
          ball.vel = new Translation3d(-ball.vel.getX() * cor, ball.vel.getY(), ball.vel.getZ());
        }
      }
    } else if (ball.pos.getX() > FIELD_LENGTH - BALL_RADIUS) {
      if (z < ALLIANCE_WALL_HEIGHT) {
        ball.pos = new Translation3d(FIELD_LENGTH - BALL_RADIUS, ball.pos.getY(), ball.pos.getZ());
        if (ball.vel.getX() > 0) {
          applyWallSpinTransfer(ball, AXIS_X_NEG);
          double cor = effectiveCOR(COR_WALL, Math.abs(ball.vel.getX()));
          ball.vel = new Translation3d(-ball.vel.getX() * cor, ball.vel.getY(), ball.vel.getZ());
        }
      }
    }

    // Y walls (guardrails: polycarbonate on aluminum extrusion, height-aware)
    if (ball.pos.getY() < BALL_RADIUS) {
      if (z < GUARDRAIL_HEIGHT) {
        ball.pos = new Translation3d(ball.pos.getX(), BALL_RADIUS, ball.pos.getZ());
        if (ball.vel.getY() < 0) {
          applyWallSpinTransfer(ball, AXIS_Y_POS);
          double cor = effectiveCOR(COR_WALL, Math.abs(ball.vel.getY()));
          ball.vel = new Translation3d(ball.vel.getX(), -ball.vel.getY() * cor, ball.vel.getZ());
        }
      }
    } else if (ball.pos.getY() > FIELD_WIDTH - BALL_RADIUS) {
      if (z < GUARDRAIL_HEIGHT) {
        ball.pos = new Translation3d(ball.pos.getX(), FIELD_WIDTH - BALL_RADIUS, ball.pos.getZ());
        if (ball.vel.getY() > 0) {
          applyWallSpinTransfer(ball, AXIS_Y_NEG);
          double cor = effectiveCOR(COR_WALL, Math.abs(ball.vel.getY()));
          ball.vel = new Translation3d(ball.vel.getX(), -ball.vel.getY() * cor, ball.vel.getZ());
        }
      }
    }
  }

  private void handleGroundContact(SimBall ball, double subDt) {
    if (ball.pos.getZ() < BALL_RADIUS) {
      ball.pos = new Translation3d(ball.pos.getX(), ball.pos.getY(), BALL_RADIUS);

      if (ball.vel.getZ() < 0) {
        double cor = effectiveCOR(COR_CARPET, Math.abs(ball.vel.getZ()));

        // If bounce would be very small, just zero out vertical velocity
        if (Math.abs(ball.vel.getZ() * cor) < 0.05) {
          ball.vel = new Translation3d(ball.vel.getX(), ball.vel.getY(), 0);
        } else {
          ball.vel = new Translation3d(ball.vel.getX(), ball.vel.getY(), -ball.vel.getZ() * cor);
        }
      }

      // Ground friction: uses surface velocity (vel + omega x r) so spin-on-contact works
      if (config.frictionEnabled) {
        double surfVelX = ball.vel.getX() - ball.omega.getY() * BALL_RADIUS;
        double surfVelY = ball.vel.getY() + ball.omega.getX() * BALL_RADIUS;
        double surfSpeed = Math.sqrt(surfVelX * surfVelX + surfVelY * surfVelY);
        double hSpeed =
            Math.sqrt(ball.vel.getX() * ball.vel.getX() + ball.vel.getY() * ball.vel.getY());

        if (surfSpeed > 0.01) {
          // friction opposes surface velocity
          double fdx = -surfVelX / surfSpeed;
          double fdy = -surfVelY / surfSpeed;

          double maxImpulse = (2.0 / 7.0) * BALL_MASS * surfSpeed;
          double frictionImpulse =
              Math.min(MU_GROUND_KINETIC * BALL_MASS * GRAVITY * subDt, maxImpulse);

          // Friction changes both linear and angular velocity
          ball.vel =
              new Translation3d(
                  ball.vel.getX() + fdx * frictionImpulse / BALL_MASS,
                  ball.vel.getY() + fdy * frictionImpulse / BALL_MASS,
                  ball.vel.getZ());
          if (config.spinTransferEnabled) {
            // Torque
            ball.omega =
                new Translation3d(
                    ball.omega.getX()
                        + BALL_RADIUS * fdy * frictionImpulse / BALL_MOMENT_OF_INERTIA,
                    ball.omega.getY()
                        - BALL_RADIUS * fdx * frictionImpulse / BALL_MOMENT_OF_INERTIA,
                    ball.omega.getZ());
          }
        } else if (hSpeed > 1e-4) {
          // Rolling resistance decelerates vel and omega together
          double decel = MU_GROUND_ROLLING * GRAVITY * subDt;
          double scale = Math.max(0, hSpeed - decel) / hSpeed;
          ball.vel =
              new Translation3d(ball.vel.getX() * scale, ball.vel.getY() * scale, ball.vel.getZ());
          if (config.spinTransferEnabled) {
            ball.omega =
                new Translation3d(
                    ball.omega.getX() * scale, ball.omega.getY() * scale, ball.omega.getZ());
          }
        }
      }

      // Settle near-zero vertical velocity when on ground
      if (Math.abs(ball.vel.getZ()) < 0.05 && ball.pos.getZ() <= BALL_RADIUS + 0.01) {
        ball.vel = new Translation3d(ball.vel.getX(), ball.vel.getY(), 0);
      }
    }
  }

  /** Handle collisions with the tent-shaped bump ramps. */
  private void handleBumpCollisions(SimBall ball) {
    for (BumpSegment seg : BUMP_SEGMENTS) {
      if (ball.pos.getY() < seg.yStart() || ball.pos.getY() > seg.yEnd()) continue;

      // Parametric projection onto line segment
      double px = ball.pos.getX() - seg.xStart();
      double pz = ball.pos.getZ() - seg.zStart();
      double t = (px * seg.lineX() + pz * seg.lineZ()) / (seg.lineLen() * seg.lineLen());
      if (t < 0 || t > 1) continue;

      // Distance from ball center to nearest point on line (in XZ plane)
      double nearX = seg.xStart() + t * seg.lineX();
      double nearZ = seg.zStart() + t * seg.lineZ();
      double dx = ball.pos.getX() - nearX;
      double dz = ball.pos.getZ() - nearZ;
      double dist = Math.sqrt(dx * dx + dz * dz);

      if (dist < BALL_RADIUS) {
        double nx = seg.nx(), nz = seg.nz();

        // Push out
        ball.pos =
            ball.pos.plus(
                new Translation3d(nx * (BALL_RADIUS - dist), 0, nz * (BALL_RADIUS - dist)));

        // Velocity reflection
        double vDotN = ball.vel.getX() * nx + ball.vel.getZ() * nz;
        if (vDotN < 0) {
          double cor = effectiveCOR(COR_HDPE, Math.abs(vDotN));
          ball.vel =
              ball.vel.minus(new Translation3d(nx * (1 + cor) * vDotN, 0, nz * (1 + cor) * vDotN));
        }
      }
    }
  }

  // Hub scoring

  private void handleHubScoring(SimBall ball) {
    if (blueHub.didScore(ball)) {
      ball.pos = blueHub.exit;
      ball.vel = blueHub.getDispersalVelocity(rng);
      ball.omega = new Translation3d();
      blueHub.score++;
      totalScored++;
    } else if (redHub.didScore(ball)) {
      ball.pos = redHub.exit;
      ball.vel = redHub.getDispersalVelocity(rng);
      ball.omega = new Translation3d();
      redHub.score++;
      totalScored++;
    }
  }

  /** Hub net collision: catches overshots, but lets balls pass through from behind. */
  private void handleNetCollision(SimBall ball, ScoringTarget hub) {
    if (ball.pos.getZ() > NET_HEIGHT_MAX || ball.pos.getZ() < NET_HEIGHT_MIN) return;
    if (ball.pos.getY() > hub.center.getY() + NET_WIDTH / 2.0
        || ball.pos.getY() < hub.center.getY() - NET_WIDTH / 2.0) return;

    double netX = hub.center.getX() + NET_OFFSET * hub.exitVelXSign;
    double distToNet = ball.pos.getX() - netX;

    if (Math.abs(distToNet) < BALL_RADIUS) {
      // Only block balls moving toward the net
      boolean movingTowardNet = ball.vel.getX() * hub.exitVelXSign > 0;
      if (!movingTowardNet) return;

      // Push ball out of net
      double pushDir = distToNet >= 0 ? 1 : -1;
      ball.pos = new Translation3d(netX + pushDir * BALL_RADIUS, ball.pos.getY(), ball.pos.getZ());
      ball.vel =
          new Translation3d(-ball.vel.getX() * COR_NET, ball.vel.getY() * COR_NET, ball.vel.getZ());
    }
  }

  private void handleRobotCollision(SimBall ball, Pose2d robotPose, Translation2d robotVel) {
    if (ball.pos.getZ() > bumperHeight) return;

    Translation2d relPos =
        new Pose2d(ball.pos.toTranslation2d(), Rotation2d.kZero)
            .relativeTo(robotPose)
            .getTranslation();

    double halfL = robotLength / 2.0 + BALL_RADIUS;
    double halfW = robotWidth / 2.0 + BALL_RADIUS;

    if (relPos.getX() < -halfL
        || relPos.getX() > halfL
        || relPos.getY() < -halfW
        || relPos.getY() > halfW) {
      return;
    }

    // Find nearest face and push out
    double dxMin = relPos.getX() + halfL;
    double dxMax = halfL - relPos.getX();
    double dyMin = relPos.getY() + halfW;
    double dyMax = halfW - relPos.getY();

    double minDist = dxMin;
    Translation2d pushDir = new Translation2d(-1, 0);

    if (dxMax < minDist) {
      minDist = dxMax;
      pushDir = new Translation2d(1, 0);
    }
    if (dyMin < minDist) {
      minDist = dyMin;
      pushDir = new Translation2d(0, -1);
    }
    if (dyMax < minDist) {
      minDist = dyMax;
      pushDir = new Translation2d(0, 1);
    }

    // Rotate push direction back to field frame
    pushDir = pushDir.rotateBy(robotPose.getRotation());
    ball.pos = ball.pos.plus(new Translation3d(pushDir.times(minDist)));

    // Velocity reflection
    Translation3d normal3d = new Translation3d(pushDir.getX(), pushDir.getY(), 0);
    double vDotN = ball.vel.toTranslation2d().dot(pushDir);
    double robotVDotN = robotVel.dot(pushDir);
    double closingVel = vDotN - robotVDotN;

    if (closingVel < 0) {
      ball.vel = ball.vel.minus(normal3d.times((1 + COR_BUMPER) * closingVel));
    }
  }

  private void handleIntakePickup(SimBall ball, Pose2d robotPose) {
    for (IntakeZone intake : intakes) {
      if (intake.shouldIntake(ball, robotPose, bumperHeight)) {
        ball.intaked = true;
        totalIntaked++;
        return;
      }
    }
  }

  // Sleeping

  private void updateSleepState(SimBall ball) {
    double speed = ball.vel.getNorm();
    double omegaMag = ball.omega.getNorm();

    if (speed < config.sleepVelocityThreshold
        && omegaMag < 0.1
        && ball.pos.getZ() <= BALL_RADIUS + 0.01) {
      ball.sleepCounter++;
      if (ball.sleepCounter >= config.sleepFrameThreshold) {
        ball.sleeping = true;
        ball.vel = new Translation3d();
        ball.omega = new Translation3d();
      }
    } else {
      ball.sleepCounter = 0;
      ball.sleeping = false;
    }
  }

  private void wakeBall(SimBall ball) {
    ball.sleeping = false;
    ball.sleepCounter = 0;
  }

  /** Wake up sleeping balls near a point (so launched balls disturb resting ones). */
  private void wakeNearbyBalls(Translation3d pos, double radius) {
    double radiusSq = radius * radius;
    for (SimBall ball : balls) {
      if (ball.sleeping) {
        double dx = ball.pos.getX() - pos.getX();
        double dy = ball.pos.getY() - pos.getY();
        double dz = ball.pos.getZ() - pos.getZ();
        if (dx * dx + dy * dy + dz * dz < radiusSq) {
          wakeBall(ball);
        }
      }
    }
  }

  // Conservation monitor

  private void computeConservationQuantities() {
    totalKE = 0;
    totalPE = 0;
    double mx = 0, my = 0, mz = 0;

    for (SimBall ball : balls) {
      double speed = ball.vel.getNorm();
      totalKE += 0.5 * BALL_MASS * speed * speed;

      // Rotational KE
      double omegaMag = ball.omega.getNorm();
      totalKE += 0.5 * BALL_MOMENT_OF_INERTIA * omegaMag * omegaMag;

      // Gravitational PE (relative to ground)
      totalPE += BALL_MASS * GRAVITY * ball.pos.getZ();

      // Linear momentum
      mx += BALL_MASS * ball.vel.getX();
      my += BALL_MASS * ball.vel.getY();
      mz += BALL_MASS * ball.vel.getZ();
    }

    totalMomentum = new Translation3d(mx, my, mz);
  }

  /** 3D cross product. Using Translation3d directly to avoid WPILib Vector conversion. */
  private static Translation3d cross(Translation3d a, Translation3d b) {
    return new Translation3d(
        a.getY() * b.getZ() - a.getZ() * b.getY(),
        a.getZ() * b.getX() - a.getX() * b.getZ(),
        a.getX() * b.getY() - a.getY() * b.getX());
  }

  /** COR drops at higher impact speeds (balls deform more and lose more energy). */
  private double velocityCOR(double e0, double impactSpeed) {
    if (!config.velocityDependentCOR) return e0;
    if (impactSpeed <= COR_VREF) return e0;
    return e0 * Math.pow(COR_VREF / impactSpeed, COR_EXPONENT);
  }

  /** Get the COR to use, with optional velocity-dependent scaling. */
  private double effectiveCOR(double baseCOR, double impactSpeed) {
    if (config.velocityDependentCOR) {
      return velocityCOR(baseCOR, impactSpeed);
    }
    return baseCOR;
  }

  /** When a ball hits a wall, friction changes its spin. This handles that. */
  private void applyWallSpinTransfer(SimBall ball, Translation3d wallNormal) {
    if (!config.spinTransferEnabled || !config.frictionEnabled) return;

    // Compute tangential velocity at contact point
    Translation3d rContact = wallNormal.times(-BALL_RADIUS);
    Translation3d surfaceVel = ball.vel.plus(cross(ball.omega, rContact));

    // Remove normal component
    double vn = surfaceVel.dot(wallNormal);
    Translation3d vTangent = surfaceVel.minus(wallNormal.times(vn));
    double vTangentMag = vTangent.getNorm();

    if (vTangentMag > 1e-4) {
      // Total normal impulse includes restitution: j_n = m * |v_n| * (1 + e)
      double impactSpeed = Math.abs(ball.vel.dot(wallNormal));
      double cor = effectiveCOR(COR_WALL, impactSpeed);
      double normalImpulse = BALL_MASS * impactSpeed * (1.0 + cor);
      double frictionImpulse = Math.min(MU_WALL * normalImpulse, BALL_MASS * vTangentMag);
      Translation3d frictionDir = vTangent.div(vTangentMag).unaryMinus();

      // Friction affects both linear velocity and spin (Newton's 3rd law)
      ball.vel = ball.vel.plus(frictionDir.times(frictionImpulse / BALL_MASS));
      Translation3d torqueImpulse = cross(rContact, frictionDir.times(frictionImpulse));
      ball.omega = ball.omega.plus(torqueImpulse.div(BALL_MOMENT_OF_INERTIA));
    }
  }

  /** Grab a contact from the pool (grows the pool if we run out). */
  private Contact allocateContact() {
    if (contactPoolIndex >= contactPool.size()) {
      Contact c = new Contact();
      contactPool.add(c);
      contactPoolIndex++;
      return c;
    }
    return contactPool.get(contactPoolIndex++);
  }

  /** Clean up balls that got eaten by intakes or flew out of bounds. */
  private void removeFlaggedBalls() {
    balls.removeIf(b -> b.intaked || b.outOfBounds);
  }

  // Trajectory prediction

  /**
   * Predict where a shot will go (gravity + drag only, no collisions). Returns points you can plot
   * in AdvantageScope Field3d. Stops at the ground.
   */
  private Translation3d[] predictArc(Translation3d pos, Translation3d vel, int steps, double dt) {
    List<Translation3d> arc = new ArrayList<>();
    arc.add(pos);
    double px = pos.getX(), py = pos.getY(), pz = pos.getZ();
    double vx = vel.getX(), vy = vel.getY(), vz = vel.getZ();
    for (int i = 0; i < steps; i++) {
      double speed = Math.sqrt(vx * vx + vy * vy + vz * vz);
      double drag = config.dragEnabled && speed > 1e-6 ? DRAG_ACCEL_FACTOR * speed : 0;
      vx -= drag * vx * dt;
      vy -= drag * vy * dt;
      vz -= (GRAVITY + drag * vz) * dt;
      px += vx * dt;
      py += vy * dt;
      pz += vz * dt;
      if (pz < BALL_RADIUS) break;
      arc.add(new Translation3d(px, py, pz));
    }
    return arc.toArray(new Translation3d[0]);
  }

  /** Push ball positions and sim stats to NetworkTables for visualization. */
  public void publishPositions() {
    // All ball positions (for Field3d rendering)
    Translation3d[] positions = new Translation3d[balls.size()];
    for (int i = 0; i < balls.size(); i++) {
      positions[i] = balls.get(i).pos;
    }
    positionPublisher.set(positions);

    // In-flight positions only (airborne balls, can render separately in Field3d)
    List<Translation3d> inFlight = new ArrayList<>();
    int sleeping = 0;
    for (SimBall b : balls) {
      if (b.intaked || b.outOfBounds) continue;
      if (b.pos.getZ() > BALL_RADIUS + 0.1) {
        inFlight.add(b.pos);
      }
      if (b.sleeping) sleeping++;
    }
    inFlightPublisher.set(inFlight.toArray(new Translation3d[0]));

    // Last shot arc (predicted trajectory visible in Field3d)
    lastShotArcPublisher.set(lastShotArc);

    // Scoring
    blueScorePub.set(blueHub.score);
    redScorePub.set(redHub.score);

    // Stats
    ballCountPub.set(balls.size());
    activeBallsPub.set(balls.size() - sleeping);
    sleepingBallsPub.set(sleeping);
    contactCountPub.set(contacts.size());
    physicsTimePub.set(lastPhysicsNanos / 1_000_000.0);
    computeConservationQuantities();
    totalEnergyPub.set(totalKE + totalPE);
  }

  public int getBallCount() {
    return balls.size();
  }

  // Only counts balls above ground level + small margin
  public int getBallsInFlight() {
    int count = 0;
    for (SimBall ball : balls) {
      if (ball.pos.getZ() > BALL_RADIUS + 0.1) count++;
    }
    return count;
  }

  public int getBallsOnGround() {
    int count = 0;
    for (SimBall ball : balls) {
      if (ball.pos.getZ() <= BALL_RADIUS + 0.1) count++;
    }
    return count;
  }

  public List<Translation3d> getBallPositions() {
    List<Translation3d> positions = new ArrayList<>(balls.size());
    for (SimBall ball : balls) {
      positions.add(ball.pos);
    }
    return positions;
  }

  public List<Translation3d> getBallVelocities() {
    List<Translation3d> velocities = new ArrayList<>(balls.size());
    for (SimBall ball : balls) {
      velocities.add(ball.vel);
    }
    return velocities;
  }

  public List<Translation3d> getBallOmegas() {
    List<Translation3d> omegas = new ArrayList<>(balls.size());
    for (SimBall ball : balls) {
      omegas.add(ball.omega);
    }
    return omegas;
  }

  public PhysicsConfig getConfig() {
    return config;
  }

  public void setConfig(PhysicsConfig config) {
    this.config = config;
    if (config.deterministic) {
      this.rng = new Random(config.deterministicSeed);
    }
  }

  /** Lock the RNG seed so tests are repeatable. */
  public void setDeterministic(long seed) {
    config.deterministic = true;
    config.deterministicSeed = seed;
    this.rng = new Random(seed);
  }

  // Translational + rotational KE
  public double getTotalKineticEnergy() {
    computeConservationQuantities();
    return totalKE;
  }

  public double getTotalPotentialEnergy() {
    computeConservationQuantities();
    return totalPE;
  }

  public Translation3d getTotalMomentum() {
    computeConservationQuantities();
    return totalMomentum;
  }

  public int getTotalLaunched() {
    return totalLaunched;
  }

  public int getTotalScored() {
    return totalScored;
  }

  public int getTotalIntaked() {
    return totalIntaked;
  }

  public double getLastLaunchSpeed() {
    return lastLaunchSpeed;
  }

  public int getBlueScore() {
    return blueHub.score;
  }

  public int getRedScore() {
    return redHub.score;
  }

  public ScoringTarget getBlueHub() {
    return blueHub;
  }

  public ScoringTarget getRedHub() {
    return redHub;
  }

  // Package-private, for tests
  List<SimBall> getBalls() {
    return balls;
  }

  public int getSleepingBallCount() {
    int count = 0;
    for (SimBall ball : balls) {
      if (ball.sleeping) count++;
    }
    return count;
  }

  public static double getFieldLength() {
    return FIELD_LENGTH;
  }

  public static double getFieldWidth() {
    return FIELD_WIDTH;
  }

  public static double getBallRadius() {
    return BALL_RADIUS;
  }

  public static double getBallMass() {
    return BALL_MASS;
  }

  public static double getMomentOfInertia() {
    return BALL_MOMENT_OF_INERTIA;
  }

  public static double getDragAccelFactor() {
    return DRAG_ACCEL_FACTOR;
  }

  public static double getMagnusAccelFactor() {
    return MAGNUS_ACCEL_FACTOR;
  }

  public static double getFieldCOR() {
    return COR_CARPET;
  }

  public static double getBallBallCOR() {
    return COR_BALL_BALL;
  }

  public void resetCounters() {
    totalLaunched = 0;
    totalScored = 0;
    totalIntaked = 0;
    lastLaunchSpeed = 0;
    blueHub.resetScore();
    redHub.resetScore();
  }
}
