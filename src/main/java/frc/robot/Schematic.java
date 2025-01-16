package frc.robot;

public class Schematic {
  // Drive (CAN IDs)
  public static final int pigeonCanId = 9;

  public static final int frontLeftDriveCanId = 1;
  public static final int backLeftDriveCanId = 3;
  public static final int frontRightDriveCanId = 5;
  public static final int backRightDriveCanId = 7;

  public static final int frontLeftTurnCanId = 2;
  public static final int backLeftTurnCanId = 4;
  public static final int frontRightTurnCanId = 6;
  public static final int backRightTurnCanId = 8;

  public static final int frontLeftAbsoluteEncoderCanId = 10;
  public static final int backLeftAbsoluteEncoderCanId = 11;
  public static final int frontRightAbsoluteEncoderCanId = 12;
  public static final int backRightAbsoluteEncoderCanId = 13;

  // Algae Effector (Motor IDs)
  public static final int PIVOT_ID = 1;
  public static final int REMOVAL_ID = 2;

  static {
    switch (Constants.getRobot()) {
      
    }
  }
}
