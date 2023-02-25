package frc.robot.commands.intakerelease;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOInputsAutoLogged;
import frc.robot.subsystems.arm.ArmIOPhysical;
import frc.robot.subsystems.intakerelease.IntakeRelease;
import frc.robot.subsystems.led.Led2023;
import frc.robot.subsystems.led.Led2023.COLORS_467;

public class ReleaseCMD extends CommandBase {
  private final IntakeRelease intakerelease;
  private final Led2023 ledStrip;
  private final Arm arm;
  
  public ReleaseCMD(IntakeRelease intakerelease, Led2023 ledStrip, Arm arm) {
    this.intakerelease = intakerelease;
    this.ledStrip = ledStrip;
    this.arm = arm;

    addRequirements(intakerelease, ledStrip, arm);
  }

  @Override
  public void execute() {
    if((!arm.hasDropped()) && intakerelease.haveCone() && (arm.getExtention() >=0.18 && arm.getRotation()>=0.1)) {
      arm.drop();
      return;
    }
    if (intakerelease.haveCube()) {
      ledStrip.setColorMovingDown(
          COLORS_467.Black.getColor(), COLORS_467.Purple.getColor()); // Purple,black
    } else if (intakerelease.haveCone()) {

      ledStrip.setColorMovingDown(
          COLORS_467.Black.getColor(), COLORS_467.Gold.getColor()); // Gold, black
    } else {
      ledStrip.setColorMovingDownTwoClr(COLORS_467.Gold.getColor(), COLORS_467.Purple.getColor());
    }
    intakerelease.release();
  }
}
