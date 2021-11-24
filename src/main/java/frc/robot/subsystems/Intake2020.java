package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.drive.MotorType;
import frc.robot.drive.SpeedControllerEncoder;
import frc.robot.drive.SpeedControllerFactory;


public class Intake2020 extends SubsystemBase {

    private SpeedControllerEncoder arm = SpeedControllerFactory.create(RobotConstants.get().intake2020ArmMotorID(), MotorType.TALON_SRX);
    private SpeedControllerEncoder roller = SpeedControllerFactory.create(RobotConstants.get().intake2020RollerMotorID(), MotorType.TALON_SRX);
    
    private boolean armIsDown = false;

    public Intake2020() {
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

    /**
    * set roller motor to regular speed so that it can pick up balls and place them into indexing
    */
    public void grabberIn() {
      System.out.println("Arm status: " + armIsDown);
      if(armIsDown) {
        roller.set(RobotConstants.get().intake2020RollerForwardSpeed());
      } 

    }

    /**
    * set roller motor to reverse so that it can unstuck any balls
    */
    public void grabberOut() {
      System.out.println("Arm status: " + armIsDown);
      if(armIsDown) {
        roller.set(-RobotConstants.get().intake2020RollerBackwardSpeed());
      }

    }

    /**
    * turn off roller motor so that it does not grab balls
    */
    public void stopGrabber() {
      roller.set(0);


    }
    @Override
    public void periodic() {

    }

}