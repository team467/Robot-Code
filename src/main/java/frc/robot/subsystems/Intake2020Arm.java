package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.motors.MotorType;
import frc.robot.motors.MotorControllerEncoder;
import frc.robot.motors.MotorControllerFactory;

/** 
 * A subsystem that deals with the arm of the intake system.
*/
public class Intake2020Arm extends SubsystemBase {

    private MotorControllerEncoder arm = MotorControllerFactory.create(RobotConstants.get().intake2020ArmMotorID(), MotorType.TALON_SRX);

    public Intake2020Arm() {
      super();
    }

    public void lowerArm() {

      arm.set(-RobotConstants.get().intake2020ArmDownSpeed());
      Intake2020Roller.armIsDown = true;

    }

    /**
    * Raises the arm so that it's in the robot perimeter for game start or when playing defense.
    */
    public void raiseArm() {

      arm.set(RobotConstants.get().intake2020ArmUpSpeed());
      Intake2020Roller.armIsDown = false;

    }

    @Override
    public void periodic() {

    }

}