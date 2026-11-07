package frc.robot;

import static org.wpilib.units.Units.RPM;

import org.wpilib.units.measure.AngularVelocity;

public class Bindings {

  public static boolean outtake = false;
  public static boolean shoot = false;
  public static boolean intake = false;
  public static boolean intakeRetract = false;

  public static AngularVelocity shooterSetpoint = RPM.zero();
  public static boolean shooterIsShooting = false;
}
