package frc.robot.commands.intakerelease;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intakerelease.IntakeRelease;
import frc.robot.subsystems.intakerelease.IntakeRelease.Wants;
import frc.robot.subsystems.led.Led2023;
import frc.robot.subsystems.led.Led2023.ColorScheme;

public class HoldCMD extends CommandBase {
  private final IntakeRelease intakerelease;
  private final Led2023 ledStrip;

  public HoldCMD(IntakeRelease intakerelease, Led2023 ledStrip) {
    this.intakerelease = intakerelease;
    this.ledStrip = ledStrip;
    addRequirements(intakerelease);
  }

  @Override
  public void execute() {
    if (intakerelease.haveCube()
        && !intakerelease.haveCone()
        && intakerelease.getWants() == Wants.CUBE) {
      ledStrip.setCmdColorScheme(ColorScheme.HOLD_CUBE);
      intakerelease.holdCube();
    } else if (intakerelease.haveCone() && intakerelease.getWants() == Wants.CONE) {
      ledStrip.setCmdColorScheme(ColorScheme.HOLD_CONE);
      intakerelease.holdCone();
    } else {
      //      ledStrip.setCmdColorScheme(ColorScheme.DEFAULT);
      intakerelease.stop();
    }
  }

  @Override
  public void end(boolean interrupted) {
    ledStrip.defaultLights();
  }

  public boolean isFinished() {
    return intakerelease.wantsCone() || intakerelease.wantsCube();
  }
}
