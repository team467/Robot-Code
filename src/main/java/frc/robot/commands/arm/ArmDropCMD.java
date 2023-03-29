package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import java.util.function.Supplier;

public class ArmDropCMD extends CommandBase {
  private static final double ROTATE_DROP_METERS = 0.025;
  private Arm arm;
  private Supplier<Boolean> haveCone;
  private Supplier<Boolean> wantsCone;
  private Timer timer = new Timer();

  public ArmDropCMD(Supplier<Boolean> haveCone, Supplier<Boolean> wantsCone, Arm arm) {
    this.arm = arm;
    this.haveCone = haveCone;
    this.wantsCone = wantsCone;
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    if (haveCone.get()
        && wantsCone.get()
        && arm.getExtention() >= 0.2
        && arm.getRotation() >= 0.1) {
      arm.setTargetPositionRotate(arm.getRotation() - ROTATE_DROP_METERS);
    }
  }

  @Override
  public void end(boolean interrupted) {
    arm.hold();
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(0.5) || arm.isFinished();
  }
}
