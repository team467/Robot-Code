package frc.robot.commands.leds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.utils.AllianceFlipUtil;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.led.Led2023;
import frc.robot.subsystems.led.Led2023.ColorScheme;

public class LedResetPoseCMD extends LedBaseCMD {
  private Drive drive;

  public LedResetPoseCMD(Led2023 ledStrip, Drive drive) {
    super(ledStrip);
    this.drive = drive;
    addRequirements(drive);
  }

  @Override
  public void execute() {
    ledStrip.setCmdColorScheme(ColorScheme.RESET_POSE);
    drive.setPose(new Pose2d(new Translation2d(), AllianceFlipUtil.apply(new Rotation2d())));
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
