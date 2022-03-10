package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.HubCameraLED;

public class HubCameraLEDEnable extends CommandBase {
    private final HubCameraLED hubCameraLED;

    public HubCameraLEDEnable(HubCameraLED hubCameraLED) {
        this.hubCameraLED = hubCameraLED;

        addRequirements(hubCameraLED);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        hubCameraLED.enable();
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
