package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.holonomictrajectory.Waypoint;
import frc.robot.FieldConstants;
import frc.robot.commands.arm.ArmCalibrateCMD;
import frc.robot.commands.drive.GoToTrajectory;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intakerelease.IntakeRelease;
import frc.robot.subsystems.led.Led2023;
import java.util.List;

public class ScoreDriveFowardBallance extends SequentialCommandGroup {
  /** Creates a new DrivelessScore. */
  public ScoreDriveFowardBallance(
      Drive drive, Arm arm, IntakeRelease intakeRelease, Led2023 ledStrip) {
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(
        new ArmCalibrateCMD(arm),
        new ScoreConeHigh(drive, arm, intakeRelease, ledStrip),
        new GoToTrajectory(
            drive,
            List.of(
                new Waypoint(
                    new Translation2d(
                        FieldConstants.Community.innerX + 0.5,
                        FieldConstants.aprilTags
                            .get(7)
                            .getTranslation()
                            .toTranslation2d()
                            .getY())))),
        new Balancing(drive));
    // add in driveless score

  }
}
