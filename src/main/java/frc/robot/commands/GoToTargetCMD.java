package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gyro;

public class GoToTargetCMD extends CommandBase {
    private final Drivetrain drivetrain;
    private final Gyro gyro;
    private GoToDistanceAngleCMD command;
    private boolean isValid;

    public GoToTargetCMD(Drivetrain drivetrain, Gyro gyro) {
        this.drivetrain = drivetrain;
        this.gyro = gyro;
    }

    @Override
    public void initialize() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("Vision").getSubTable("HubTarget");
        isValid = table.getEntry("isValid").getBoolean(false);

        if (isValid) {
            command = new GoToDistanceAngleCMD(drivetrain, gyro, -(Units.feetToMeters(table.getEntry("distance").getDouble(0)) - 1.0), table.getEntry("angle").getDouble(0), true);
            command.schedule();
        }
    }

    @Override
    public boolean isFinished() {
        return command.isFinished() || !isValid;
    }

}
