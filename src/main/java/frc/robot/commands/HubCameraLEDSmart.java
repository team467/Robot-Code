package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.HubCameraLED;
import frc.robot.subsystems.LlamaNeck2022;
import frc.robot.subsystems.Shooter2022;

public class HubCameraLEDSmart extends CommandBase {
    private HubCameraLED hubCameraLED;
    private Shooter2022 shooter;

    public HubCameraLEDSmart(HubCameraLED hubCameraLED, Shooter2022 shooter) {
        this.hubCameraLED = hubCameraLED;
        this.shooter = shooter;

        addRequirements(hubCameraLED);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (shooter.llamaNeck2022.upperLimitSwitchIsPressed() || shooter.getCurrentCommand() instanceof Shooter2022ShootCMD || shooter.getCurrentCommand() instanceof Shooter2022ShootSpeedCMD  || shooter.getCurrentCommand() instanceof Shooter2022ShootTargetCMD) {
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
