package frc.robot.subsystems.stereoVision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class stereoVision extends SubsystemBase {
  private final stereoVisionIO io;
  private final stereoVisionInputsAutoLogged inputs;

  public stereoVision(stereoVisionIO io) {
    this.io = io;
    this.inputs = new stereoVisionInputsAutoLogged();
  }

  @Override
  public void periodic() {}
}
