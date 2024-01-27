package frc.robot.subsystems.robotstate;

import edu.wpi.first.wpilibj.DriverStation;

public class RobotStateIOPhysical implements RobotStateIO {

  public void initialize(RobotStateIOInputs inputs) {
    if (DriverStation.getAlliance().isPresent()) {
      inputs.alliance = DriverStation.getAlliance().get();
    }
  }

  public void updateInputs(RobotStateIOInputs inputs) {
    // Update estop state
    if (DriverStation.isEStopped()) {
      inputs.estopped = true;
    } else {
      inputs.estopped = false;
    }

    if (DriverStation.isDisabled()) {
      inputs.disabled = true;
    } else {
      inputs.disabled = false;
    }
  }
}
