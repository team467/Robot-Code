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
  public static final int shooterFrontLeftCanId;
  public static final int shooterFrontRightCanId;
  public static final int shooterBackCanId;
  // hopper belt sub-system
  public static final int hopperBeltCanId;

  //  indexer subsystem
  public static final int indexerIndexCanId;
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

        // Shooter (CAN Ids)
        shooterFrontLeftCanId = 0;
        shooterFrontRightCanId = 0;
        shooterBackCanId = 0;

        // Hopper Belt (CAN Ids)
        hopperBeltCanId = 0;

        // Indexer (CAN IDs)
        indexerIndexCanId = 0;
        indexerFeedupCanId = 0;

        // Intake (CAN IDs)
        intakeMotorCanId = 1;
        intakeExtendCanId = 2;

        // Climber (CAN IDs)
        climberMotorCanId = 0;
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
        shooterFrontLeftCanId = 0;
        shooterFrontRightCanId = 0;
        shooterBackCanId = 0;

        // Hopper Belt (CAN Ids)
        hopperBeltCanId = 0;

        // Indexer (CAN IDs)
        indexerIndexCanId = 0;
        indexerFeedupCanId = 0;

        // Intake (CAN IDs)
        intakeMotorCanId = 1;
        intakeExtendCanId = 2;

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
        shooterFrontLeftCanId = 0;
        shooterFrontRightCanId = 0;
        shooterBackCanId = 0;

        // Hopper Belt (CAN Ids)
        hopperBeltCanId = 0;

        // Indexer (CAN IDs)
        indexerIndexCanId = 0;
        indexerFeedupCanId = 0;

        // Intake (CAN IDs)
        intakeMotorCanId = 1;
        intakeExtendCanId = 2;

        // Climber (CAN IDs)
        climberMotorCanId = 0;
      }
    }
  }
}
