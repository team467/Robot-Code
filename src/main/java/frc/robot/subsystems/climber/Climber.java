package frc.robot.subsystems.climber;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {

  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs;

  private NetworkTable climberTable;
  private NetworkTableEntry climberTestingEntry;

  public Climber(ClimberIO io) {
    this.io = io;
    inputs = new ClimberIOInputsAutoLogged();

    climberTable = NetworkTableInstance.getDefault().getTable("Climber");
    climberTestingEntry = climberTable.getEntry("Speed");
    climberTestingEntry.setDouble(0.0);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
    RobotState.getInstance().climberStowed = inputs.climberStowed;

    if (DriverStation.isTest()) {
      io.setSpeed(climberTestingEntry.getDouble(0.0));
    }
  }

  /**
   * Command to deploy the climber.
   *
   * @return the deploy command.
   */
  public Command deploy() {
    return Commands.run(
            () -> {
              io.setSpeed(1.0);
              RobotState.getInstance().climberDeployed = true;
            },
            this)
        .until(() -> inputs.position >= ClimberConstants.DEPLOYED_POSITION);
  }

  /**
   * Command to winch the climber.
   *
   * @return the winch command.
   */
  public Command winch() {
    return Commands.run(
            () -> {
              io.setSpeed(-1.0);
              RobotState.getInstance().climberWinched = true;
            },
            this)
        .until(() -> inputs.position <= ClimberConstants.WINCHED_POSITION);
  }
}
