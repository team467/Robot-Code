package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import java.awt.*;

public class Event {
  private boolean state = false;
  private double condition;
  private double distance;
  private Command executable;

  public Event(double condition, Command executable) {
    this.condition = condition;
    this.executable = executable;
  }

  public void setState(boolean state) {
    this.state = state;
  }
  /*If condition is set to -1  it will trigger at start of the straightDriveToPose command*/
  public void setCondition(double condition) {
    this.condition = condition;
  }

  public void checkTrigger(double distance) {
    if (distance <= condition && !state) {
      state = true;
      executable.execute();
    }
  }

  public void Trigger() {
    state = true;
    executable.schedule();
  }

  public boolean getState() {
    return state;
  }

  public double getCondition() {
    return condition;
  }

  public void nullType() {
    state = true;
  }
}
