package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotConstants;
import frc.robot.subsystems.LEDTower2022;

public class LEDTower2022FoundBallCMD extends CommandBase {
    private final double TIMER_SPEED = 0.5;

    private LEDTower2022 ledTower;
    private Color teamColor = Color.kBlue;
    private boolean lit = true;
    private Timer timer = new Timer();

    public LEDTower2022FoundBallCMD(LEDTower2022 ledTower) {
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

        lit = true;
        timer.reset();
    }

    @Override
    public void execute() {
        if (timer.hasElapsed(TIMER_SPEED)) {
            lit = !lit;
            timer.reset();
        }

        Color color = Color.kBlack;
        if (lit) color = teamColor;

        for (int i = 0; i < RobotConstants.get().ledTower2022LEDCount(); i++) {
            ledTower.setLED(i, color);
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

