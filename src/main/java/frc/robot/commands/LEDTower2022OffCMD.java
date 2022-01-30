package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotConstants;
import frc.robot.subsystems.LEDTower2022;

public class LEDTower2022OffCMD extends CommandBase {
    private LEDTower2022 ledTower;

    public LEDTower2022OffCMD(LEDTower2022 ledTower) {
        this.ledTower = ledTower;

        addRequirements(ledTower);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        for (int i = 0; i < RobotConstants.get().ledTower2022LEDCount(); i++) {
            ledTower.setLED(i, Color.kBlack);
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

