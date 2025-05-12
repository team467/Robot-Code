package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.awt.*;

public class Event {
  private boolean state = false;
  private Trigger condition;
  private double distance;
  private final Command executable;

  public Event(Trigger condition, Command executable) {
    this.condition = condition;
    this.executable = executable;
  }

  public void setState(boolean state) {
    this.state = state;
  }
  /*If condition is set to -1  it will trigger at start of the straightDriveToPose command*/
  public void setCondition(boolean condition) {
    this.condition = new Trigger(() -> condition);
  }

  public void checkTrigger() {
    condition.getAsBoolean();
  }

  public void Trigger() {
    state = true;
    executable.execute();
  }

  public boolean getState() {
    return state;
  }

  public Trigger getCondition() {
    return condition;
  }

  public void nullType() {
    state = true;
  }
}
