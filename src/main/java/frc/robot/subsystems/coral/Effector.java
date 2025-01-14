package frc.robot.subsystems.coral;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;

public class Effector extends SubsystemBase {

  private final EffectorIO io;

  private final RobotState robotState = RobotState.getInstance();
  private boolean PIDMode = false;
  private double currentVelocitySetpoint;

  // private SimpleMotorFeedforward shooterFeedforward =SF;
  // new

  public Effector(EffectorIO io) {
    this.io = io;
  }
}
