package first.robot.opmodes.teleop;

import first.robot.Orchestrator;
import first.robot.Robot;
import first.robot.constants.FieldConstants;
import first.robot.constants.IntakeConstants;
import first.robot.mechanisms.shooter.ShooterConstants;
import first.robot.util.CustomTriggers;
import org.wpilib.command3.Command;
import org.wpilib.command3.Trigger;
import org.wpilib.command3.button.CommandGamepad;
import org.wpilib.driverstation.GenericHID;
import org.wpilib.driverstation.GenericHID.RumbleType;
import org.wpilib.driverstation.POVDirection;

final class TeleopBindings {
  private TeleopBindings() {}

  static void bindCommon(Robot robot) {
    CommandGamepad driver = robot.driverController;
    CommandGamepad operator = robot.operatorController;
    Orchestrator orchestrator = robot.orchestrator;

    robot.drive.setDefaultCommand(robot.drive.driverControl(driver));

    Trigger intaking = new Trigger(robot.intake::isIntaking);
    intaking.onTrue(rumble(driver, 0.2));
    intaking.onFalse(rumble(driver, 0.0));

    driver.start().onTrue(robot.drive.resetHeading());
    new Trigger(() -> driver.getHID().getHID().getPOV() != POVDirection.CENTER)
        .whileTrue(robot.drive.driveWithDpad(() -> driver.getHID().getHID().getPOV()));
    driver.faceLeft().toggleOnTrue(orchestrator.zoneBasedAim());
    driver.faceUp().toggleOnTrue(robot.intake.extendToAngleAndIntake(IntakeConstants.COLLAPSE_POS));

    driver.rightTrigger(0.1).whileTrue(orchestrator.shoot());
    driver.leftTrigger(0.2).toggleOnTrue(robot.intake.runIntakeMotor());

    driver
        .rightStick()
        .whileTrue(
            robot.drive.aimAssist(
                driver, () -> FieldConstants.targetHub().orElseGet(robot.drive::getPose)));

    operator
        .leftTrigger(0.1)
        .toggleOnTrue(orchestrator.spinUpShooterDistance(orchestrator.getHubDistance()));
    operator
        .rightTrigger(0.1)
        .and(operator.dpadUp().negate())
        .toggleOnTrue(orchestrator.zoneBasedShooter());
    operator.rightTrigger(0.1).and(operator.dpadUp()).toggleOnTrue(orchestrator.spinUpShooterHub());
    operator.faceUp().whileTrue(robot.indexer.reverse());
    operator.faceLeft().whileTrue(robot.intake.outtakeRollers());
    operator.dpadRight().whileTrue(robot.shooter.setVoltage(ShooterConstants.KS + 1));
  }

  static void bindAutoMode(Robot robot) {
    CommandGamepad driver = robot.driverController;

    CustomTriggers.toggleIntakeUp(driver.leftBumper(), robot.intake::isDeployed)
        .and(() -> !robot.shooter.isAtSetpoint())
        .toggleOnTrue(robot.intake.extendToAngle(IntakeConstants.COLLAPSE_POS));
    CustomTriggers.toggleIntakeUp(driver.leftBumper(), robot.intake::isDeployed)
        .and(robot.shooter::isAtSetpoint)
        .toggleOnTrue(robot.intake.slowlyBringInIntakeWithoutRollers());
    CustomTriggers.toggleIntakeDown(driver.leftBumper(), () -> !robot.intake.isDeployed())
        .toggleOnTrue(robot.intake.extendToAngle(IntakeConstants.EXTEND_POS));

    driver.rightBumper().toggleOnTrue(robot.orchestrator.driveToHub());
  }

  static void bindManualMode(Robot robot) {
    CommandGamepad driver = robot.driverController;

    driver.leftBumper().whileTrue(robot.intake.runIntakeExtendVolts(-4));
    driver.leftBumper().onFalse(robot.intake.stopExtendingCommand());
    driver.rightBumper().whileTrue(robot.intake.runIntakeExtendVolts(4));
    driver.rightBumper().onFalse(robot.intake.stopExtendingCommand());

    driver.faceDown().onTrue(robot.intake.resetExtendPosition());
  }

  private static Command rumble(CommandGamepad controller, double value) {
    return Command.noRequirements(
            coroutine -> {
              GenericHID hid = controller.getHID().getHID();
              hid.setRumble(RumbleType.LEFT_RUMBLE, value);
              hid.setRumble(RumbleType.RIGHT_RUMBLE, value);
            })
        .named("Rumble[" + value + "]");
  }
}
