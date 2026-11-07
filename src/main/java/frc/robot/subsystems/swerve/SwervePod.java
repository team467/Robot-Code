package frc.robot.subsystems.swerve;

import org.wpilib.epilogue.Logged;
import org.wpilib.math.kinematics.SwerveModulePosition;

@Logged
public class SwervePod<DriveMotorT, TurnMotorT> {

  SwervePod(DriveMotorT driveMotor, TurnMotorT turnMotor, ID id) {
  }

  SwerveModulePosition getPosition() {
    return null;
  }

  public enum ID {
    FRONT_LEFT,
    FRONT_RIGHT,
    BACK_LEFT,
    BACK_RIGHT
  }
}
