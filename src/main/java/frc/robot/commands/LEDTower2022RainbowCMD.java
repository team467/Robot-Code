package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotConstants;
import frc.robot.subsystems.LEDTower2022;

public class LEDTower2022RainbowCMD extends CommandBase {
    private final double TIMER_SPEED = 0.01;

    private LEDTower2022 ledTower;
    private int color = 0;
    private Timer timer = new Timer();

    public LEDTower2022RainbowCMD(LEDTower2022 ledTower) {
        this.ledTower = ledTower;
        timer.start();

        addRequirements(ledTower);
    }

    @Override
    public void initialize() {
        timer.reset();
    }

    @Override
    public void execute() {
        if (timer.hasElapsed(TIMER_SPEED)) {
            color += 1;

            if (color > 180) color = 0;
            timer.reset();
        }
        
        for (int i = 0; i < RobotConstants.get().ledTower2022LEDCount(); i++) {
            ledTower.setHSV(i, color, 255, 127);
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