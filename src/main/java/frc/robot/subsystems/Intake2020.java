package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.drive.MotorType;
import frc.robot.drive.SpeedControllerEncoder;
import frc.robot.drive.SpeedControllerFactory;


public class Intake2020 extends SubsystemBase {

    private boolean enabled = false;

    private SpeedControllerEncoder arm = SpeedControllerFactory.create(RobotConstants.get().intake2020ArmMotorID(), MotorType.TALON_SRX);
    private SpeedControllerEncoder roller = SpeedControllerFactory.create(RobotConstants.get().intake2020RollerMotorID(), MotorType.TALON_SRX);
    
    Boolean down = false;

    public Intake2020() {
      super();
    }

    public boolean isEnabled() {
      return enabled;
    }

    //stage 1

      /**
       * Lowers the arm so that it can pick up balls.
      */
      public void lowerArm() {

      arm.set(RobotConstants.get().intake2020ArmDownSpeed());
      //down = true;

    }

    /**
    * Raises the arm so that it's in the robot permeter for game start or when playing defense.
    */
    public void raiseGrabber() {

      arm.set(RobotConstants.get().intake2020ArmUpSpeed());
      //down = false;

    }

    //stage 2
    /**
    * set roller motor to regular speed so that it can pick up balls and place them into indexing
    */
    public void forwardRoller() {

      if(down) {
        roller.set(RobotConstants.get().intake2020RollerForwardSpeed());  
      }  

    }

    /**
    * set roller motor to reverse so that it can unstuck any balls
    */
    public void reverseRoller() {

      if(down) {
        roller.set(-RobotConstants.get().intake2020RollerBackwardSpeed());
      }

    }

    /**
    * turn off roller motor so that it does not grab balls
    */
    public void stopRoller() {
      
      roller.set(0);

    }

}

    