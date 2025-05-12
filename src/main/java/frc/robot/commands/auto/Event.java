package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.awt.*;
import java.util.function.Supplier;

public class Event {
  private boolean state = false;
  private double condition;
  private double distance;
  private Command executable;
  public Event (double condition, double distance, Command executable) {
    this.condition = condition;
    this.distance = distance;
    this.executable = executable;
  }
  public void setState(boolean state) {
    this.state = state;
  }
  public void setCondition(double condition) {
    this.condition = condition;
  }
  public void checkTrigger(double distance) {
    if (distance <= condition && !state) {
      state = true;
      executable.schedule();
    }
  }

}
