package frc.robot.commands.intakerelease;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intakerelease.IntakeRelease;
import frc.robot.subsystems.led.Led2023;
import frc.robot.subsystems.led.Led2023.COLORS_467;

public class HoldCMD extends CommandBase {
  private final IntakeRelease intakerelease;
  private final Led2023 ledStrip;

  public HoldCMD(IntakeRelease intakerelease, Led2023 ledStrip) {
    this.intakerelease = intakerelease;
    this.ledStrip = ledStrip;

    addRequirements(intakerelease, ledStrip);
  }

  @Override
  public void execute() {
    if (intakerelease.haveCube() && !intakerelease.haveCone()) {
      ledStrip.setBlinkColors(COLORS_467.Purple, COLORS_467.Purple, COLORS_467.White.getColor());
      intakerelease.holdCube();
    } else if (intakerelease.haveCone()) {
      ledStrip.setBlinkColors(COLORS_467.Yellow, COLORS_467.Yellow, COLORS_467.White.getColor());
      intakerelease.holdCone();
    } else {
      intakerelease.stop();
    }
  }

  @Override
  public void end(boolean interrupted) {
    ledStrip.defaultLights();
  }
}
