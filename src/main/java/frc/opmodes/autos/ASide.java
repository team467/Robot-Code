package frc.opmodes.autos;

import static org.wpilib.units.Units.Seconds;

import frc.robot.Robot;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.PeriodicOpMode;

@Autonomous(name = "A-side", group = "pathplanner")
public class ASide extends PeriodicOpMode {

  private final Robot robot;

  public ASide(Robot r) {
    this.robot = r;
  }

  @Override
  public void start() {
    Scheduler.getDefault().schedule(Command.noRequirements(
        coro -> {
          robot.initPose();

          var prepareShoot = Command.noRequirements(coro2 -> {
                coro2.wait(Seconds.of(5));
                robot.prepareToShoot();
              }
          ).named("A-side Prepare Shoot");

          coro.awaitAny(
              robot.followPath("path-name"), // The only one that can finish
              robot.collectBalls(),
              prepareShoot
          );

          coro.await(robot.shootAtHub());

          coro.awaitAny(
              robot.followPath("path-2"),
              robot.collectBalls(),
              prepareShoot
          );

          coro.await(robot.shootAtHub());
        }
    ).named("Left Side Auto"));
  }
}
