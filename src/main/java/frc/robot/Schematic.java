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

  // shooter subsystem
  public static final int shooterBottomMotorCanId;
  public static final int shooterTopMotorCanId;
  public static final int shooterMiddleMotorCanId;
  // magic carpet sub-system
  public static final int magicCarpetCanId;

  //  indexer subsystem
  public static final int indexerFeedupCanId;

  // intake subsystem
  public static final int intakeMotorCanId;
  public static final int intakeExtendCanId;

  // climber subsystem
  public static final int climberMotorCanId;

  static {
    switch (Constants.getRobot()) {
      case ROBOT_2025_COMP -> {
        // Device CAN IDs
        // TODO: Change for 2025 as needed
        // Drive (CAN IDs)
        pigeonCanId = 17;

        frontLeftDriveCanId = 3;
        frontLeftTurnCanId = 4;
        frontRightDriveCanId = 7;
        frontRightTurnCanId = 8;

        backRightDriveCanId = 1;
        backRightTurnCanId = 2;
        backLeftDriveCanId = 5;
        backLeftTurnCanId = 6;

        frontLeftAbsoluteEncoderCanId = 19;
        frontRightAbsoluteEncoderCanId = 21;
        backRightAbsoluteEncoderCanId = 18;
        backLeftAbsoluteEncoderCanId = 20;

        // Shooter (CAN Ids)
        shooterBottomMotorCanId = 0;
        shooterTopMotorCanId = 0;
        shooterMiddleMotorCanId = 0;

        // Magic Carpet (CAN Ids)
        magicCarpetCanId = 0;

        // Indexer (CAN IDs and DIOs)
        indexerFeedupCanId = 0;

        // Intake (CAN IDs)
        intakeMotorCanId = 0;
        intakeExtendCanId = 0;

        // Climber (CAN IDs)
        climberMotorCanId = 0;
      }

      case ROBOT_2026_COMP -> {
        // Device CAN Id
        pigeonCanId = 17;

        // Drive (CAN Ids)
        frontLeftDriveCanId = 1;
        frontLeftTurnCanId = 2;
        frontRightDriveCanId = 3;
        frontRightTurnCanId = 4;

        backRightDriveCanId = 5;
        backRightTurnCanId = 6;
        backLeftDriveCanId = 7;
        backLeftTurnCanId = 8;

        frontLeftAbsoluteEncoderCanId = 18;
        frontRightAbsoluteEncoderCanId = 19;
        backRightAbsoluteEncoderCanId = 21;
        backLeftAbsoluteEncoderCanId = 20;

        // Shooter (CAN Ids)
        shooterTopMotorCanId = 11;
        shooterMiddleMotorCanId = 12;
        shooterBottomMotorCanId = 13;

        // Hopper (CAN Ids)
        magicCarpetCanId = 15;

        // Indexer (CAN Ids)
        indexerFeedupCanId = 9;

        // Intake (CAN IDs)
        intakeMotorCanId = 23;
        intakeExtendCanId = 14;

        // Climber (CAN Ids)
        climberMotorCanId = 22;
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

        // Shooter (CAN Ids)
        shooterBottomMotorCanId = 0;
        shooterTopMotorCanId = 0;
        shooterMiddleMotorCanId = 0;

        // Magic Carpet (CAN Ids)
        magicCarpetCanId = 0;

        // Indexer (CAN IDs and DIOs)
        indexerFeedupCanId = 0;

        // Intake (CAN IDs)
        intakeMotorCanId = 0;
        intakeExtendCanId = 0;

        // Climber (CAN IDs)
        climberMotorCanId = 0;
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

        // Shooter (CAN Ids)
        shooterBottomMotorCanId = 0;
        shooterTopMotorCanId = 0;
        shooterMiddleMotorCanId = 0;

        // Magic Carpet (CAN Ids)
        magicCarpetCanId = 0;

        // Indexer (CAN IDs and DIOs)
        indexerFeedupCanId = 0;

        // Intake (CAN IDs)
        intakeMotorCanId = 0;
        intakeExtendCanId = 0;

        // Climber (CAN IDs)
        climberMotorCanId = 0;
      }
    }
  }
}
