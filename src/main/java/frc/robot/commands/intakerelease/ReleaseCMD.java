package frc.robot.commands.intakerelease;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intakerelease.IntakeRelease;
import frc.robot.subsystems.led.Led2023;
import frc.robot.subsystems.led.Led2023.COLORS_467;
import frc.robot.subsystems.led.Led2023.ColorScheme;

public class ReleaseCMD extends CommandBase {
  private final IntakeRelease intakerelease;
  private final Led2023 ledStrip;
  private final Arm arm;
  private boolean needsDrop;

  public ReleaseCMD(IntakeRelease intakerelease, Led2023 ledStrip, Arm arm) {
    this.intakerelease = intakerelease;
    this.ledStrip = ledStrip;
    this.arm = arm;

    addRequirements(intakerelease, ledStrip, arm);
  }

  @Override
  public void initialize() {
    ledStrip.set(COLORS_467.Black);
    needsDrop = true;
  }

  @Override
  public void execute() {
    if ((!arm.hasDropped())
        && intakerelease.haveCone()
        && (arm.getExtention() >= 0.18 && arm.getRotation() >= 0.1)&&needsDrop) {
      arm.drop();
      return;
    }
    needsDrop = false;
    if (intakerelease.haveCube()) {
      ledStrip.setCmdColorScheme(ColorScheme.RELEASE_CUBE);
    } else if (intakerelease.haveCone()) {
      ledStrip.setCmdColorScheme(ColorScheme.RELEASE_CONE);
    } else {
      ledStrip.setCmdColorScheme(ColorScheme.RELEASE_UNKNOWN);
    }
    intakerelease.release();
  }

  @Override
  public void end(boolean interrupted) {
    ledStrip.defaultLights();
  }
}
