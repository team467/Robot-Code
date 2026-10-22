package first.robot;

import com.ctre.phoenix6.CANBus;
import org.wpilib.hardware.bus.CANPort;

public class Schematic {
  public static final CANPort canPort = CANPort.CAN_S0;
  public static final CANBus canBus = CANBus.systemcore(canPort.value);

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

  public static final int shooterTopLeftMotorCanId;
  public static final int shooterTopRightMotorCanId;
  public static final int shooterBottomLeftMotorCanId;
  public static final int shooterBottomRightMotorCanId;
  public static final int magicCarpetCanId;

  public static final int indexerFeedupCanId;

  public static final int intakeMotorCanId;
  public static final int intakeExtendCanId;

  public static final int climberMotorCanId;

  static {
    switch (Constants.getRobot()) {
      case ROBOT_2025_COMP -> {
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

        shooterTopLeftMotorCanId = 0;
        shooterTopRightMotorCanId = 0;
        shooterBottomLeftMotorCanId = 0;
        shooterBottomRightMotorCanId = 0;

        magicCarpetCanId = 0;

        indexerFeedupCanId = 0;

        intakeMotorCanId = 0;
        intakeExtendCanId = 0;

        climberMotorCanId = 0;
      }

      case ROBOT_2026_COMP -> {
        pigeonCanId = 17;

        frontLeftDriveCanId = 5;
        frontLeftTurnCanId = 6;
        frontRightDriveCanId = 3;
        frontRightTurnCanId = 4;

        backRightDriveCanId = 1;
        backRightTurnCanId = 2;
        backLeftDriveCanId = 7;
        backLeftTurnCanId = 8;

        frontLeftAbsoluteEncoderCanId = 20;
        frontRightAbsoluteEncoderCanId = 19;
        backRightAbsoluteEncoderCanId = 18;
        backLeftAbsoluteEncoderCanId = 21;

        shooterTopLeftMotorCanId = 33;
        shooterTopRightMotorCanId = 35;
        shooterBottomLeftMotorCanId = 34;
        shooterBottomRightMotorCanId = 36;

        magicCarpetCanId = 15;

        indexerFeedupCanId = 9;

        intakeMotorCanId = 23;
        intakeExtendCanId = 14;

        climberMotorCanId = 22;
      }

      case ROBOT_BRIEFCASE -> {
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

        shooterTopLeftMotorCanId = 8;
        shooterTopRightMotorCanId = 1;
        shooterBottomLeftMotorCanId = 7;
        shooterBottomRightMotorCanId = 2;

        magicCarpetCanId = 0;

        indexerFeedupCanId = 0;

        intakeMotorCanId = 0;
        intakeExtendCanId = 0;

        climberMotorCanId = 0;
      }

      default -> {
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

        shooterTopLeftMotorCanId = 0;
        shooterTopRightMotorCanId = 0;
        shooterBottomLeftMotorCanId = 0;
        shooterBottomRightMotorCanId = 0;

        magicCarpetCanId = 0;

        indexerFeedupCanId = 0;

        intakeMotorCanId = 0;
        intakeExtendCanId = 0;

        climberMotorCanId = 0;
      }
    }
  }
}
