package frc.robot.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.RobotConstants;
import frc.robot.utilities.MathUtils;

/**
 * Button and axes assignments for an XInput Controller.
 */
public final class XboxController467 extends XboxController {
private double DEADZONE = 0.1;

    public XboxController467(int port) {
        super(port);
    }

    @Override
    public double getLeftX() {
        if (Math.abs(super.getLeftX()) < DEADZONE) return 0;
        return super.getLeftX();
    }

    @Override
    public double getLeftY() {
        if (Math.abs(super.getLeftY()) < DEADZONE) return 0;
        return super.getLeftY();
    }

    @Override
    public double getRightX() {
        if (Math.abs(super.getRightX()) < DEADZONE) return 0;
        return super.getRightX();
    }

    @Override
    public double getRightY() {
        if (Math.abs(super.getRightY()) < DEADZONE) return 0;
        return super.getRightY();
    }

    /**
     * Returns the drive speed, taking the turbo and slow triggers into account.
     */
    public double getAdjustedSpeed(final double speed) {
        if (getLeftTriggerAxis() > 0.0) {
            // For some reason, up stick is negative, so we flip it
            return turboFastSpeed(speed);
        } else {
            return turboSlowSpeed(speed);
        }
    }

    public double turboFastSpeed(final double speed) {
        // Speed multiplied by acceleration determined by left trigger
        return speed * MathUtils.weightedAverage(RobotConstants.get().driveNormalMaxSpeed(), RobotConstants.get().driveFastMaxSpeed(), getLeftTriggerAxis());
    }

    public double turboSlowSpeed(final double speed) {
        // Speed multiplied by deceleration determined by right trigger
        return speed * MathUtils.weightedAverage(RobotConstants.get().driveNormalMaxSpeed(), RobotConstants.get().driveSlowMaxSpeed(), getRightTriggerAxis());
    }

    public double getAdjustedDriveSpeed() {
        return -getAdjustedSpeed(getLeftY());
    }

    /**
     * Returns the turn speed, which is slower when the robot is driving fast.
     */
    public double getAdjustedTurnSpeed() {
        return getAdjustedSpeed(getRightX()) * MathUtils.weightedAverage(RobotConstants.get().driveNormalTurnMaxSpeed(),
            RobotConstants.get().driveSlowTurnMaxSpeed(), Math.abs(getAdjustedSpeed(getLeftY())));
    }

    /**
   * Calculate the distance of this stick from the center position.
   *
   * @return
   */
  public double getLeftStickDistance() {
    return Math.sqrt((getLeftX() * getLeftX()) + (getLeftY() * getLeftY()));
  }

  public double getRightStickDistance() {
    return Math.sqrt((getRightX() * getRightX()) + (getRightY() * getRightY()));
  }

  private double calculateStickAngle(final double stickX, final double stickY) {
    if (stickY == 0.0) {
      // In Y deadzone avoid divide by zero error
      return (stickX > 0.0) ? Math.PI / 2 : (-Math.PI) / 2;
    }

    // Return value in range -PI to PI
    double stickAngle = Math.atan2(stickX, -stickY);

    if (stickY > 0) {
      stickAngle += (stickX > 0) ? Math.PI : -Math.PI;
    }

    return (stickAngle);
  }

    public enum Buttons {
        A(1),
        B(2),
        X(3),
        Y(4),
        BumperLeft(5),
        BumperRight(6),
        Back(7),
        Start(8),
        XBox(9),
        StickLeft(10),
        StickRight(11),
        POVup(12),
        POVright(13),
        POVdown(14),
        POVleft(15);

        public final int value;

        Buttons(int value) {
            this.value = value;
        }
    }

    public enum Axes {
        LeftX(0),
        LeftY(1),
        LeftTrigger(2),
        RightTrigger(3),
        RightX(4),
        RightY(5);

        public final int value;

        Axes(int value) {
            this.value = value;
        }
    }
}
