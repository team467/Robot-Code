// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.networktables.EntryListenerFlags;
// import edu.wpi.first.wpilibj.TimedRobot;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.networktables.NetworkTableInstance;

public class Shooter2022SemiAutoShootCMD extends CommandBase {
  
  /** Creates a new Shooter2022SemiAutoShootCMD. */
  public Shooter2022SemiAutoShootCMD(Drivetrain drivetrain, Shooter2022 shooter) {
    addRequirements(drivetrain, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  // Vision Network tables?
  // @Override
  // public void robotInit() {
  //   table = NetworkTableInstance.getDefault().getTable("GRIP/mycontoursReport");
  // }

  // @Override
  // public void teleopPeriodic() {
  //     double x = table.getEntry("X");
  //     double y = table.getEntry("Y");
  //     double angle = table.getEntry("Angle");
  // Values will be used for driving hopefully.


  //     double[] areas = table.getEntry("area").getDoubleArray(defaultValue);

  // }
}
