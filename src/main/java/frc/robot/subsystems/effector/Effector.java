package frc.robot.subsystems.effector;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Effector extends SubsystemBase {
  private final EffectorIO effectorIO;
  private final EffectorIOInputsAutoLogged inputs;

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

  public Effector(EffectorIO effectorIO) {
    super();
    this.effectorIO = effectorIO;
    inputs = new EffectorIOInputsAutoLogged();
    this.effectorIO.updateInputs(inputs, Wants.CONE);
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
    effectorIO.updateInputs(inputs, wants);
    Logger.processInputs("Effector", inputs);
    Logger.recordOutput("Effector/State", state.toString());

    switch (state) {
      case DISABLED -> effectorIO.setPercent(0);
      case INTAKE -> effectorIO.setPercent(-1.0);
      case RELEASE -> effectorIO.setPercent(0.6);
      case HOLD_CUBE -> effectorIO.setPercent(inputs.cubeLimitSwitch ? -0.1 : -0.35);
      case HOLD_CONE -> effectorIO.setPercent(-0.8);
      case STOP -> effectorIO.setPercent(0);
      default -> effectorIO.setPercent(0);
    }
  }

  public void intake() {
    hasCone = false;
    hasCube = false;
    state = State.INTAKE;
  }

  public void resetHas() {
    hasCone = false;
    hasCube = false;
  }

  public void release() {
    state = State.RELEASE;
    hasCone = false;
    hasCube = false;
  }

  public void stop() {
    state = State.STOP;
    resetHas();
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

  public boolean cubeLimitSwitch() {
    return inputs.cubeLimitSwitch;
  }

  public boolean coneLimitSwitch() {
    return inputs.coneLimitSwitch;
  }

  public boolean haveCone() {
    hasCone = hasCone || inputs.coneLimitSwitch;
    return hasCone;
  }

  public boolean isFinished() {
    return (inputs.coneLimitSwitch || (Wants.CUBE == wants && inputs.cubeLimitSwitch));
  }
}
