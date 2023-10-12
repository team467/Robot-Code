package frc.robot.commands.intakerelease;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.commands.arm.ArmDropCMD;
// import frc.robot.commands.arm.ArmHomeCMD;
// import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intakerelease.IntakeRelease;

// public class ReleaseCMD extends SequentialCommandGroup {
//   public ReleaseCMD(IntakeRelease intakerelease, Arm arm) {
//     addCommands(
//         new ConditionalCommand(
//             Commands.sequence(
//                 new ArmDropCMD(intakerelease::haveCone, intakerelease::wantsCone, arm),
//                 // .withTimeout(0.4),
//                 // new WaitCommand(0.5),
//                 Commands.parallel(
//                         Commands.run(intakerelease::release, intakerelease).withTimeout(0.5),
//                         new ArmHomeCMD(arm, intakerelease::wantsCone))
//                     .withTimeout(5.0)),
//             Commands.sequence(
//                 new ArmDropCMD(intakerelease::haveCone, intakerelease::wantsCone, arm)
//                     .withTimeout(0.4),
//                 Commands.run(intakerelease::release, intakerelease).withTimeout(0.5),
//                 new ArmHomeCMD(arm, intakerelease::wantsCone).withTimeout(5.0)),
//             intakerelease::wantsCone));
//   }
// }
