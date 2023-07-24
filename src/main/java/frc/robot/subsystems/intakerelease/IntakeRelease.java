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
    return wants;
  }

  public boolean wantsCone() {
    return wants == Wants.CONE;
  }

  public boolean wantsCube() {
    return wants == Wants.CUBE;
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
    logger.recordOutput("IntakeRelease/State", state.toString());

    switch (state) {
      case DISABLED -> intakeReleaseIO.setPercent(0);
      case INTAKE -> intakeReleaseIO.setPercent(-1.0);
      case RELEASE -> intakeReleaseIO.setPercent(0.6);
      case HOLD_CUBE -> intakeReleaseIO.setPercent(inputs.cubeLimitSwitch ? -0.1 : -0.35);
      case HOLD_CONE -> intakeReleaseIO.setPercent(-0.8);
      case STOP -> intakeReleaseIO.setPercent(0);
      default -> intakeReleaseIO.setPercent(0);
    }
  }

  /** Instructs the system to actively grab a game piece */
  public void intake() {
    hasCone = false;
    hasCube = false;
    state = State.INTAKE;
  }

  /** Resets the flags indicating if the system is holding a cone or a cube game piece. */
  public void resetHas() {
    hasCone = false;
    hasCube = false;
  }

  /** Instructs the system to release whatever game piece it's holding. */
  public void release() {
    state = State.RELEASE;
    hasCone = false;
    hasCube = false;
  }

  /** Instructs the system to stop its current operation. */
  public void stop() {
    state = State.STOP;
    resetHas();
  }

  /** Instructs the system to hold ont a cone. */
  public void holdCone() {
    state = State.HOLD_CONE;
  }

  /** Instructs the system to hold onto a cube. */
  public void holdCube() {
    state = State.HOLD_CUBE;
  }

  /**
   * Checks if the system is currently holding a cube.
   *
   * @return true of a cone is detected, false if a cube is not detected
   */
  public boolean haveCube() {
    hasCube = hasCube || inputs.cubeLimitSwitch;
    return hasCube;
  }

  /**
   * Checks the status of the cube limit switch to determine if a cube is currently detected.
   *
   * @return true if a cube is detected, false if a cube is not detected
   */
  public boolean cubeLimitSwitch() {
    return inputs.cubeLimitSwitch;
  }

  /**
   * Checks the status of the cone limit switch to determine if a cone is currently detected.
   *
   * @return true if a cone is detected, false if a cone is not detected
   */
  public boolean coneLimitSwitch() {
    return inputs.coneLimitSwitch;
  }

  /**
   * Checks if the system is currently holding a cone.
   *
   * @return true of a cone is detected, false if a cone is not detected
   */
  public boolean haveCone() {
    hasCone = hasCone || inputs.coneLimitSwitch;
    return hasCone;
  }

  public boolean isFinished() {
    return (inputs.coneLimitSwitch || (Wants.CUBE == wants && inputs.cubeLimitSwitch));
  }
}
