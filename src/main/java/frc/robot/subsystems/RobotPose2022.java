package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;


public class RobotPose2022 extends SubsystemBase{
    private double angleToTarget;

    //private Robotpose2022 double;

    public double getAngleToTarget() {
        return angleToTarget;
    }

    public void updateWithCamera(double newAngleToTarget) {
        angleToTarget = newAngleToTarget; 
    }

    public void updateWithGyro(double deltaAngle) {
        //counterclockewise = positive, vice versa
        angleToTarget += deltaAngle;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        // builder.addDoubleProperty("Angle To Target", () -> newAngleToTarget.updateWithCamera() + deltaAngle.updateWithGyro(), null);
        // builder.addDoubleProperty("Camera Angle", () -> newAngleToTarget.updateWithCamera(), null);
        // builder.addDoubleProperty("Gyro Delta Angle", () -> deltaAngle.updateWithGyro(), null);

        builder.addDoubleProperty("Angle To Target", () -> newAngleToTargt.updateWithCamera() + deltaAngle.updateWithGyro(), null);
        builder.addDoubleProperty("Camera Angle", () -> newAngleToTarget.updateWithCamera(), null);
        builder.addDoubleProperty("Gyro Delta Angle", () -> deltaAngle.updateWithGyro(), null);
    }
}