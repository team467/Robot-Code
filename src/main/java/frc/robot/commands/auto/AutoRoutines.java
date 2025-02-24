package frc.robot.commands.auto;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import frc.lib.utils.AllianceFlipUtil;
import frc.robot.subsystems.drive.Drive;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class AutoRoutines {
  private final Drive drive;
  private final AutoFactory autoFactory;

  public AutoRoutines(Drive drive) {
    this.drive = drive;
    this.autoFactory =
        new AutoFactory(
            drive::getPose,
            drive::setPose,
            drive::followTrajectory,
            true,
            drive,
            this::logTrajectory);
  }

  private void logTrajectory(Trajectory<SwerveSample> trajectory, boolean isFinished) {
    Logger.recordOutput(
        "Odometry/Trajectory",
        AllianceFlipUtil.shouldFlip()
            ? trajectory.flipped().samples().toArray(new SwerveSample[0])
            : trajectory.samples().toArray(new SwerveSample[0]));
    Logger.recordOutput("Odometry/TrajectoryFinished", isFinished);
  }

  private AutoRoutine leaveRoutine(String startingPoint) {
    String name = startingPoint + " Leave";
    var routine = autoFactory.newRoutine(name);
    var trajectory = routine.trajectory(name);

    routine.active().onTrue(trajectory.resetOdometry().andThen(trajectory.cmd()));
    return routine;
  }

  private AutoRoutine test(String startingPoint) {
    String name = startingPoint;
    var routine = autoFactory.newRoutine(name);
    var trajectory = routine.trajectory(name);

    routine.active().onTrue(trajectory.resetOdometry().andThen(trajectory.cmd()));
    return routine;
  }

  private AutoRoutine createScoringSequence(String name, String[] trajectories) {
    var routine = autoFactory.newRoutine(name);

    var startTrajectory = routine.trajectory(trajectories[0]);
    routine.active().onTrue(startTrajectory.resetOdometry().andThen(startTrajectory.cmd()));
    // TODO score
    var previousTrajectory = startTrajectory;
    for (String trajectoryName :
        java.util.Arrays.stream(trajectories).skip(1).toArray(String[]::new)) {
      var trajectory = routine.trajectory(trajectoryName);
      previousTrajectory.done().onTrue(trajectory.cmd());
      if (trajectoryName.endsWith("S")) {
        // TODO intake
      } else {
        // TODO score
      }
      previousTrajectory = trajectory;
    }

    return routine;
  }

  public Map<String, AutoRoutine> getRoutines() {
    Map<String, AutoRoutine> autoRoutines = new HashMap<>();

    // Adding routines
    autoRoutines.put("A leave", leaveRoutine("A"));
    autoRoutines.put("B leave", leaveRoutine("B"));
    autoRoutines.put("C leave", leaveRoutine("C"));

    // Adding scoring sequences
    autoRoutines.put("B1L", createScoringSequence("B1L", new String[] {"B1L"}));
    autoRoutines.put("B1R", createScoringSequence("B1R", new String[] {"B1R"}));
    autoRoutines.put("A2R3LR", test("A2R3LR"));
    // createScoringSequence("A2R3LR", new String[] {"A2R", "2RS", "S3L", "3LS", "S3R"}));
    autoRoutines.put(
        "A3LR4L",
        createScoringSequence("A3LR4L", new String[] {"A3L", "3LS", "S3R", "3RS", "S4L"}));
    autoRoutines.put(
        "C5RL4R",
        createScoringSequence("C5RL4R", new String[] {"C5R", "5RS", "S5L", "5LS", "S4R"}));
    autoRoutines.put(
        "C6L5RL",
        createScoringSequence("C6L5RL", new String[] {"C6L", "6LS", "S5R", "5RS", "S5L"}));
    autoRoutines.put(
        "B1R2LR",
        createScoringSequence("B1R2LR", new String[] {"B1R", "1RS", "S2L", "2LS", "S2R"}));
    autoRoutines.put(
        "B1L6RL",
        createScoringSequence("B1L6RL", new String[] {"B1L", "1LS", "S6R", "6RS", "S6L"}));

    return autoRoutines;
  }
}
