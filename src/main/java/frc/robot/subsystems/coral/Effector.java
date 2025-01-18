package frc.robot.subsystems.coral;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

public class CoralEffector extends SubsystemBase {

  private EffectorIO io;
  private EffectorIOInputsAutoLogged inputs = new EffectorIOInputsAutoLogged();
  private final RobotState robotState = RobotState.getInstance();
  private boolean PIDMode = false;
  private double currentVelocitySetpoint;

  public CoralEffector(EffectorIO io) {
    this.io = io;
    this.inputs = new EffectorIOInputsAutoLogged();
  }

  public void Periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("CoralEffector", inputs);
  }

  public Command setEffectorVolts(double volts) {
    return Commands.run(
        () -> {
          io.setVoltage(volts);
        },
        this);
  }

  public void setCoralOnTheWay(boolean coralOnTheWayState) {
    io.setCoralOnTheWay(coralOnTheWayState);
  }

  //  public void setLimitSwitch(){
  //     if(inputs.coralOnTheWay == true){
  //         while(inputs.coralGone == false) {
  //             io.setVoltage(12);
  //         }
  //     }
  //  }

  // ADD TEMPERATURE and OTHERS; Fix problems of getting using Limit Switch that is not in the motor

}


