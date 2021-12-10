package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter2020;

public class ShooterSetCMD extends CommandBase {
    private Shooter2020 shooter2020;
    private Supplier<Double> speedSupplier;

    public ShooterSetCMD(Shooter2020 shooter2020, Supplier<Double> speedSupplier) {
        this.shooter2020 = shooter2020;
        this.speedSupplier = speedSupplier;

        addRequirements(shooter2020);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        shooter2020.setFlywheelRaw(speedSupplier.get());
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
