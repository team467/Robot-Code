package frc.lib.utils;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;

/** Represents a subsystem unit that requires a periodic callback but not a hardware mutex. */
public abstract class VirtualSubsystem extends SubsystemBase {
  private static List<VirtualSubsystem> subsystems = new ArrayList<>();

  public VirtualSubsystem() {
    subsystems.add(this);
  }

  public static void periodicAll() {
    for (VirtualSubsystem subsystem : subsystems) {
      subsystem.periodic();
    }
  }

  public abstract void periodic();
}
