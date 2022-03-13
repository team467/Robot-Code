package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.HubCameraLED;
import frc.robot.subsystems.LlamaNeck2022;

public class HubCameraLEDSmart extends CommandBase {
    private HubCameraLED hubCameraLED;
    private LlamaNeck2022 llamaNeck;

    public HubCameraLEDSmart(HubCameraLED hubCameraLED, LlamaNeck2022 llamaNeck) {
        this.hubCameraLED = hubCameraLED;
        this.llamaNeck = llamaNeck;

        addRequirements(hubCameraLED);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (llamaNeck.upperLimitSwitchIsPressed()) {
            hubCameraLED.enable();
        } else {
            hubCameraLED.disable();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
