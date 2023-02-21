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
    STOP;
  }

  private State state;

  public IntakeRelease(IntakeReleaseIO intakeReleaseIO) {
    super();
    this.intakeReleaseIO = intakeReleaseIO;
    inputs = new IntakeReleaseIOInputsAutoLogged();
    this.intakeReleaseIO.updateInputs(inputs);
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
    intakeReleaseIO.updateInputs(inputs);
    logger.processInputs("IntakeRelease", inputs);

    switch (state) {
      case DISABLED:
        intakeReleaseIO.setVelocity(0);
        break;

      case INTAKE:
        intakeReleaseIO.setVelocity(0.3);
        break;

      case RELEASE:
        intakeReleaseIO.setVelocity(-0.3);
        break;
      case HOLD_CUBE:
        intakeReleaseIO.setVelocity(-0.01);
        break;
      case HOLD_CONE:
        intakeReleaseIO.setVelocity(-0.01);
        break;
      case STOP:
      default:
        intakeReleaseIO.setVelocity(0);
        break;
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
