package frc.opmodes;

import frc.robot.Robot;
import org.wpilib.command3.Command;
import org.wpilib.command3.button.CommandGamepad;
import org.wpilib.opmode.OpMode;
import org.wpilib.opmode.Teleop;

@Teleop(group = "TeleOp", name = "Primary TeleOp")
public class TeleOp implements OpMode {

  CommandGamepad driverController = new CommandGamepad(0);
  CommandGamepad operatorController = new CommandGamepad(1);

  public TeleOp(Robot r) {
    driverController.faceLeft().onTrue(
        Command.noRequirements(
            _ -> r.shoot()
        ).named("Shoot Enable")
    ).onFalse(Command.noRequirements(_ -> r.stopShooting()).named("Shoot Disable"));

    driverController.rightTrigger(0.1).onTrue(
        Command.noRequirements(
            _ -> r.intake()
        ).named("Intake Enable")
    ).onFalse(Command.noRequirements(_ -> r.stopIntaking()).named("Intake Disable"));

    operatorController.faceLeft().onTrue(
        Command.noRequirements(
            _ -> r.outtake()
        ).named("Outtake Enable")
    ).onFalse(Command.noRequirements(_ -> r.stopOuttake()).named("Outtake Disable"));

    operatorController.dpadDown().onTrue(
        Command.noRequirements(
            _ -> r.shootManual()
        ).named("Shoot Manual Enable")
    ).onFalse(Command.noRequirements(_ -> r.stopShooting()).named("Shoot Disable"));

    operatorController.dpadDown().and(driverController.rightBumper()).onTrue(
        Command.noRequirements(
            _ -> r.retractIntake()
        ).named("Retract Intake Enable")
    ).onFalse(
        Command.noRequirements(_ -> r.stopRetractingIntake()).named("Retract Intake Disable"));

    operatorController.dpadDown().negate().and(driverController.rightBumper()).onTrue(
        Command.noRequirements(
            _ -> r.driveToHub()
        ).named("Extend Intake Enable")
    );
  }
}
