package frc.robot.subsystems.coral;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;

public class Effector extends SubsystemBase {

  private final EffectorIO io;
  //private final EffectorIOInputsAutoLogged inputs;

  private final RobotState robotState = RobotState.getInstance();
  private boolean PIDMode = false;
  private double currentVelocitySetpoint;

  private SimpleMotorFeedforward shooterDropforward






 /* Not Sure To Use AutoLogged

 public Effector(EffectorIO io) {

    this.io = io;
    this.inputs = new EffectorIOInputsAutoLogged();
  }
  */

  public Effector(EffectorIO io){
    this.io = io;

  }



}
