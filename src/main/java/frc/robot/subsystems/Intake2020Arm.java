package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.drive.MotorType;
import frc.robot.drive.SpeedControllerEncoder;
import frc.robot.drive.SpeedControllerFactory;


public class Intake2020Arm extends SubsystemBase {

    private SpeedControllerEncoder arm = SpeedControllerFactory.create(RobotConstants.get().intake2020ArmMotorID(), MotorType.TALON_SRX);
    
    private boolean armIsDown = false;

    public Intake2020Arm() {
      super();
    }

    public void lowerArm() {

      arm.set(-RobotConstants.get().intake2020ArmDownSpeed());
      armIsDown = true;

    }

    /**
    * Raises the arm so that it's in the robot perimeter for game start or when playing defense.
    */
    public void raiseArm() {

      arm.set(RobotConstants.get().intake2020ArmUpSpeed());
      armIsDown = false;

    }

    @Override
    public void periodic() {

    }

}