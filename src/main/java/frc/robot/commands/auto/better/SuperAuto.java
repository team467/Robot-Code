package frc.robot.commands.auto.better;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;

public class SuperAuto extends SequentialCommandGroup {
  public SuperAuto(Drive drive, AutoOptions options) {}

  public enum AutoOptions {
    LEFT_OF_CHARGE(0b1),
    RIGHT_OF_CHARGE(0b10),
    LOCATION_A(0b100),
    LOCATION_B(0b1000),
    LOCATION_C(0b10000),
    LOCATION_D(0b100000);

    public final int value;

    AutoOptions(int value) {
      this.value = value;
    }
  }
}
