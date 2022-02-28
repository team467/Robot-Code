package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class PuppyModeCMD extends CommandBase {
    private final double BALL_TIMEOUT = 0.1;

    private Drivetrain drivetrain;
    private Timer timer;
    private NetworkTableEntry hasBallEntry; // bool
    private NetworkTableEntry distanceEntry; // double
    private NetworkTableEntry angleEntry; // double

    public PuppyModeCMD(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("Vision").getSubTable("BallTracking");
        NetworkTable colorTable;
        timer = new Timer();
        timer.start();
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

        hasBallEntry = colorTable.getEntry("HasBall");
        distanceEntry = colorTable.getEntry("Distance");
        angleEntry = colorTable.getEntry("Angle");

        hasBallEntry.setBoolean(false);
        distanceEntry.setDouble(0);
        angleEntry.setDouble(0);

        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.reset();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (hasBallEntry.getBoolean(false)) {
            double angle = angleEntry.getDouble(0)/120; // 30deg range
            double speed = MathUtil.clamp(distanceEntry.getDouble(0) / 120, 0.1, 0.5); // 120 inches max distance

            // drivetrain.curvatureDrive(speed, angle);
            drivetrain.curvatureDrive(0, angle);
            timer.reset();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivetrain.arcadeDrive(0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(BALL_TIMEOUT);
    }
}
