package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class ArcadeDriveCMD extends CommandBase {
    private Drivetrain drivetrain;
    private Supplier<Double> speedSupplier;
    private Supplier<Double> rotateSupplier;

    public ArcadeDriveCMD(Drivetrain drivetrain, Supplier<Double> speedSupplier, Supplier<Double> rotateSupplier) {
        this.drivetrain = drivetrain;
        this.speedSupplier = speedSupplier;
        this.rotateSupplier = rotateSupplier;

        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("Init Arcade");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        System.out.println(String.format("Arcade speed: %f, Arcade rotate: %f", speedSupplier.get(), rotateSupplier.get()));
        drivetrain.arcadeDrive(speedSupplier.get(), rotateSupplier.get());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("End Arcade");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
