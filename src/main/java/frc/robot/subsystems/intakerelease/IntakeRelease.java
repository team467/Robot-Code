package frc.robot.subsystems.intakerelease;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakeRelease extends SubsystemBase {
  private final Logger logger = Logger.getInstance();
  private final IntakeReleaseIO intakeReleaseIO;
  private final IntakeReleaseIOInputsAutoLogged inputs;

  private enum State {
    DISABLED,
    INTAKE,
    RELEASE,
    HOLD_CUBE,
    HOLD_CONE,
    STOP
  }

  private State state;

  public IntakeRelease(IntakeReleaseIO intakeReleaseIO) {
    super();
    this.intakeReleaseIO = intakeReleaseIO;
    inputs = new IntakeReleaseIOInputsAutoLogged();
    this.intakeReleaseIO.updateInputs(inputs, Wants.NONE);
    state = State.STOP;
  }

  public enum Wants {
    CONE,
    CUBE,
    NONE
  }

  private Wants wants = Wants.NONE;

  public Wants getWants() {
    return (wants);
  }

  public void setWants(Wants wants) {
    this.wants = wants;
  }

  @Override
  public void periodic() {
    intakeReleaseIO.updateInputs(inputs, wants);
    logger.processInputs("IntakeRelease", inputs);

    switch (state) {
      case DISABLED -> intakeReleaseIO.setPercent(0);
      case INTAKE -> intakeReleaseIO.setPercent(-0.7);
      case RELEASE -> intakeReleaseIO.setPercent(0.4);
      case HOLD_CUBE -> intakeReleaseIO.setPercent(-0.1);
      case HOLD_CONE -> intakeReleaseIO.setPercent(-0.3);
      case STOP -> intakeReleaseIO.setPercent(0);
      default -> intakeReleaseIO.setPercent(0);
    }
  }

  public void intake() {
    state = State.INTAKE;
  }

  public void release() {
    state = State.RELEASE;
  }

  public void stop() {
    state = State.STOP;
  }

  public void holdCone() {
    state = State.HOLD_CONE;
  }

  public void holdCube() {
    state = State.HOLD_CUBE;
  }

  public boolean haveCube() {
    return inputs.cubeLimitSwitch;
  }

  public boolean haveCone() {
    return inputs.coneLimitSwitch;
  }

  public boolean isFinished() {
    return (inputs.coneLimitSwitch || (Wants.CUBE == wants && inputs.cubeLimitSwitch));
  }
}
