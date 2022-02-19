package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gyro;

public class GoToBallCMD extends CommandBase {
    private final Drivetrain drivetrain;
    private final Gyro gyro;
    private GoToDistanceAngleCMD command;

    public GoToBallCMD(Drivetrain drivetrain, Gyro gyro) {
        this.drivetrain = drivetrain;
        this.gyro = gyro;
    }

    @Override
    public void initialize() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("Vision").getSubTable("BallTracking");
        NetworkTable colorTable;
        switch(DriverStation.getAlliance()) {
          case Blue:
            colorTable = table.getSubTable("Blue");
            break;
          case Invalid:
          case Red:
          default:
            colorTable = table.getSubTable("Red");
            break;

        }

        command = new GoToDistanceAngleCMD(drivetrain, gyro, colorTable.getEntry("Distance").getDouble(0), colorTable.getEntry("Angle").getDouble(0), false);
        command.schedule();
    }

    @Override
    public boolean isFinished() {
        return command.isFinished();
    }

}
