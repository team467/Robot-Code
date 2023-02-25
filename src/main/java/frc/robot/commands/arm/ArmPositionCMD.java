package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPositionConstants.ArmPosition;
import frc.robot.subsystems.led.Led2023;
import java.util.function.Supplier;

public class ArmPositionCMD extends CommandBase {
  protected final Arm arm;
  protected final Supplier<ArmPosition> armPositionSupplier;
  protected final Led2023 ledStrip;

  public ArmPositionCMD(Arm arm, ArmPosition armPosition, Led2023 ledStrip) {
    this.arm = arm;
    this.armPositionSupplier = () -> armPosition;
    this.ledStrip = ledStrip;
    addRequirements(arm);
  }

  public ArmPositionCMD(Arm arm, Supplier<ArmPosition> armPosition, Led2023 ledStrip) {
    this.arm = arm;
    this.armPositionSupplier = armPosition;
    this.ledStrip = ledStrip;
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    ArmPosition armPosition = armPositionSupplier.get();
    arm.setTargetPositions(armPosition.extendSetpoint, armPosition.rotateSetpoint);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    arm.hold();
    ledStrip.defaultLights();
  }

  @Override
  public boolean isFinished() {
    return arm.isFinished();
  }
}
