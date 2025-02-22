package frc.robot;

public class Schematic {
  // Drive (CAN IDs)
  public static final int pigeonCanId;

  public static final int frontLeftDriveCanId;
  public static final int backLeftDriveCanId;
  public static final int frontRightDriveCanId;
  public static final int backRightDriveCanId;

  public static final int frontLeftTurnCanId;
  public static final int backLeftTurnCanId;
  public static final int frontRightTurnCanId;
  public static final int backRightTurnCanId;

  public static final int frontLeftAbsoluteEncoderCanId;
  public static final int backLeftAbsoluteEncoderCanId;
  public static final int frontRightAbsoluteEncoderCanId;
  public static final int backRightAbsoluteEncoderCanId;

  // Algae Effector (Motor IDs)
  public static final int algaePivotCanId;
  public static final int algaeRemovalCanId;
  // Coral Effector (Motor IDs)
  public static final int hopperReflectorSensorDioId;
  public static final int coralMotorID;
  // Elevator
  public static final int elevatorMotorID;

  static {
    switch (Constants.getRobot()) {
      case ROBOT_2025_TEST -> {
        // Drive (CAN IDs)
        pigeonCanId = 17;

        frontLeftDriveCanId = 1;
        backLeftDriveCanId = 5;
        frontRightDriveCanId = 3;
        backRightDriveCanId = 7;

        frontLeftTurnCanId = 2;
        backLeftTurnCanId = 6;
        frontRightTurnCanId = 4;
        backRightTurnCanId = 8;

        frontLeftAbsoluteEncoderCanId = 21;
        backLeftAbsoluteEncoderCanId = 20;
        frontRightAbsoluteEncoderCanId = 18;
        backRightAbsoluteEncoderCanId = 19;

        algaePivotCanId = 12;
        algaeRemovalCanId = 13;

        hopperReflectorSensorDioId = 9;
        coralMotorID = 24;

        elevatorMotorID = 9;
      }
      case ROBOT_2025_COMP -> {
        // Device CAN IDs
        // TODO: Change for 2025 as needed
        // Drive (CAN IDs)
        pigeonCanId = 17;

        frontLeftDriveCanId = 1;
        backLeftDriveCanId = 7;
        frontRightDriveCanId = 3;
        backRightDriveCanId = 5;

        frontLeftTurnCanId = 2;
        backLeftTurnCanId = 8;
        frontRightTurnCanId = 4;
        backRightTurnCanId = 6;

        frontLeftAbsoluteEncoderCanId = 18;
        backLeftAbsoluteEncoderCanId = 21;
        frontRightAbsoluteEncoderCanId = 19;
        backRightAbsoluteEncoderCanId = 20;

        // Algae Effector (Motor IDs)
        algaePivotCanId = 12;
        algaeRemovalCanId = 13;

        hopperReflectorSensorDioId = 9;
        coralMotorID = 14;

        elevatorMotorID = 9;
      }

      case ROBOT_BRIEFCASE -> {
        // Drive (CAN IDs)
        pigeonCanId = 0;

        frontLeftDriveCanId = 0;
        backLeftDriveCanId = 0;
        frontRightDriveCanId = 0;
        backRightDriveCanId = 0;

        frontLeftTurnCanId = 0;
        backLeftTurnCanId = 0;
        frontRightTurnCanId = 0;
        backRightTurnCanId = 0;

        frontLeftAbsoluteEncoderCanId = 0;
        backLeftAbsoluteEncoderCanId = 0;
        frontRightAbsoluteEncoderCanId = 0;
        backRightAbsoluteEncoderCanId = 0;

        // Algae Effector (Motor IDs)
        algaePivotCanId = 2;
        algaeRemovalCanId = 1;

        coralMotorID = 14;
        hopperReflectorSensorDioId = 9;

        elevatorMotorID = 9;
      }

      default -> {
        // Drive (CAN IDs)
        pigeonCanId = 0;

        frontLeftDriveCanId = 0;
        backLeftDriveCanId = 0;
        frontRightDriveCanId = 0;
        backRightDriveCanId = 0;

        frontLeftTurnCanId = 0;
        backLeftTurnCanId = 0;
        frontRightTurnCanId = 0;
        backRightTurnCanId = 0;

        frontLeftAbsoluteEncoderCanId = 0;
        backLeftAbsoluteEncoderCanId = 0;
        frontRightAbsoluteEncoderCanId = 0;
        backRightAbsoluteEncoderCanId = 0;

        // Algae Effector (Motor IDs)
        algaePivotCanId = 0;
        algaeRemovalCanId = 0;

        hopperReflectorSensorDioId = 0;
        coralMotorID = 0;

        elevatorMotorID = 0;
      }
    }
  }
}
