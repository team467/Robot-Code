package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotConstants;
import frc.robot.subsystems.LEDTower2022;

public class LEDTower2022ChaseBallCMD extends CommandBase {
    private final double TIMER_SPEED = 0.5;

    private LEDTower2022 ledTower;
    private Color teamColor = Color.kBlue;
    private Timer timer = new Timer();

    public LEDTower2022ChaseBallCMD(LEDTower2022 ledTower) {
        this.ledTower = ledTower;
        timer.start();

        addRequirements(ledTower);
    }

    @Override
    public void initialize() {
        if (DriverStation.getAlliance() == Alliance.Red) {
            teamColor = Color.kRed;
        } else {
            teamColor = Color.kBlue;
        }

        timer.reset();
    }

    @Override
    public void execute() {
        if (timer.hasElapsed(TIMER_SPEED * (RobotConstants.get().ledTower2022LEDCount() + 1))) {
            timer.reset();
        }

        for (int i = 0; i < RobotConstants.get().ledTower2022LEDCount(); i++) {
            if (timer.hasElapsed(TIMER_SPEED * i)) {
                double timeUntilOff = Math.max(0, (TIMER_SPEED * (i + 1)) - timer.get());
                int brightness = (int) (255 * timeUntilOff);

                ledTower.setRGB(i, (int) teamColor.red * brightness, (int) teamColor.green * brightness, (int) teamColor.blue * brightness);
            } else {
                ledTower.setRGB(i, 0, 0, 0);
            }
        }

        ledTower.sendData();
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

