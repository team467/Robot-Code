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

  // climber subsystem
  public static final int climberCanId;

  // indexer subsystem
  public static final int indexerCanId;
  public static final int indexerFeedUpCanId;

  static {
    switch (Constants.getRobot()) {
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
        backRightAbsoluteEncoderCanId = 20;
        backLeftAbsoluteEncoderCanId = 21;

        // Shooter (CAN Ids)
        shooterFrontLeftCanId = 11;
        shooterBackCanId = 12;
        shooterFrontRightCanId = 13;

        // Hopper (CAN Ids)
        hopperBeltCanId = 14;

        // Climber (CAN Ids)
        climberCanId = 15;

        // Indexer (CAN Ids)
        indexerCanId = 9;
        indexerFeedUpCanId = 10;
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

        // Climber (CAN Ids)
        climberCanId = 0;

        // Indexer (CAN Ids)
        indexerCanId = 0;
        indexerFeedUpCanId = 0;
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

        // Climber (CAN Ids)
        climberCanId = 0;

        // Indexer (CAN Ids)
        indexerCanId = 0;
        indexerFeedUpCanId = 0;
      }
    }
  }
}
