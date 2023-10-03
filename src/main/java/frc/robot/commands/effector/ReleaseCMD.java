package frc.robot.commands.effector;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ArmDropCMD;
import frc.robot.commands.arm.ArmHomeCMD;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.effector.Effector;

public class ReleaseCMD extends SequentialCommandGroup {
  public ReleaseCMD(Effector effector, Arm arm) {
    addCommands(
        new ConditionalCommand(
            Commands.sequence(
                new ArmDropCMD(effector::haveCone, effector::wantsCone, arm),
                // .withTimeout(0.4),
                // new WaitCommand(0.5),
                Commands.parallel(
                        Commands.run(effector::release, effector).withTimeout(0.5),
                        new ArmHomeCMD(arm, effector::wantsCone))
                    .withTimeout(5.0)),
            Commands.sequence(
                new ArmDropCMD(effector::haveCone, effector::wantsCone, arm)
                    .withTimeout(0.4),
                Commands.run(effector::release, effector).withTimeout(0.5),
                new ArmHomeCMD(arm, effector::wantsCone).withTimeout(5.0)),
            effector::wantsCone));
  }
}
