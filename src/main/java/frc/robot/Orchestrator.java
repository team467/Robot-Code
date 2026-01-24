package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopperbelt.HopperBelt;
import frc.robot.subsystems.shooter.Shooter;

public class Orchestrator {
  private final Drive drive;
  private final Shooter shooter;
  private final HopperBelt hopperBelt;
  private final RobotState robotState = RobotState.getInstance();

  public Orchestrator(Drive drive, HopperBelt hopperBelt, Shooter shooter) {
    this.drive = drive;
    this.hopperBelt = hopperBelt;
    this.shooter = shooter;
  }

  public Command runHopperBeltandShooter() {
    return Commands.parallel(shooter.setVoltage(1), hopperBelt.start());
  }
}
