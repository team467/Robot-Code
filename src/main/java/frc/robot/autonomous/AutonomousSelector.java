package frc.robot.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class AutonomousSelector extends SendableChooser<AutonomousMode> {
  public void setDefaultOption(AutonomousMode autonomousMode) {
    super.setDefaultOption(autonomousMode.getName(), autonomousMode);
  }

  public void addOption(AutonomousMode autonomousMode) {
    super.addOption(autonomousMode.getName(), autonomousMode);
  }
}
