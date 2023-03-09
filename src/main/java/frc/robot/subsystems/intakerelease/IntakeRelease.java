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

  private boolean hasCone = false;
  private boolean hasCube = false;

  public IntakeRelease(IntakeReleaseIO intakeReleaseIO) {
    super();
    this.intakeReleaseIO = intakeReleaseIO;
    inputs = new IntakeReleaseIOInputsAutoLogged();
    this.intakeReleaseIO.updateInputs(inputs, Wants.CONE);
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

  public boolean wantsCone() {
    return wants == Wants.CONE;
  }

  public void setWants(Wants wants) {
    this.wants = wants;
    hasCone = false;
    hasCube = false;
  }

  @Override
  public void periodic() {
    intakeReleaseIO.updateInputs(inputs, wants);
    logger.processInputs("IntakeRelease", inputs);

    switch (state) {
      case DISABLED -> intakeReleaseIO.setPercent(0);
      case INTAKE -> intakeReleaseIO.setPercent(-1.0);
      case RELEASE -> {
        intakeReleaseIO.setPercent(0.4);
        hasCone = false;
        hasCube = false;
      }
      case HOLD_CUBE -> intakeReleaseIO.setPercent(-0.3);
      case HOLD_CONE -> intakeReleaseIO.setPercent(-0.8);
      case STOP -> intakeReleaseIO.setPercent(0);
      default -> intakeReleaseIO.setPercent(0);
    }
  }

  public void intake() {
    hasCone = false;
    hasCube = false;
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
    hasCube = hasCube || inputs.cubeLimitSwitch;
    return hasCube;
  }

  public boolean haveCone() {
    hasCone = hasCone || inputs.coneLimitSwitch;
    return hasCone;
  }

  public boolean isFinished() {
    return (inputs.coneLimitSwitch || (Wants.CUBE == wants && inputs.cubeLimitSwitch));
  }
}
