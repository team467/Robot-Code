package frc.robot.subsystems.pixy2;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pixy2 extends SubsystemBase {
  private final Pixy2IO io;
  private final Pixy2IOInputsAutoLogged inputs = new Pixy2IOInputsAutoLogged();
  private static final NetworkTableInstance networkTables = NetworkTableInstance.getDefault();

  public Pixy2(Pixy2IO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }
}
