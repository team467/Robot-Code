package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.motors.MotorType;
import frc.robot.motors.SpeedControllerEncoder;
import frc.robot.motors.SpeedControllerFactory;

/** 
 * A subsystem that deals with the roller of the intake system.
*/
public class Intake2020Roller extends SubsystemBase {

    private SpeedControllerEncoder roller = SpeedControllerFactory.create(RobotConstants.get().intake2020RollerMotorID(), MotorType.TALON_SRX);
    
    public static boolean armIsDown = true;

    public Intake2020Roller() {
      super();
    }

    /**
    * set roller motor to regular speed so that it can pick up balls and place them into indexing
    */
    public void rollerIn() {
      System.out.println("Arm status: " + armIsDown);
      if(armIsDown) {
        roller.set(RobotConstants.get().intake2020RollerForwardSpeed());
      } 

    }

    /**
    * set roller motor to reverse so that it can unstuck any balls
    */
    public void rollerOut() {
      System.out.println("Arm status: " + armIsDown);
      if(armIsDown) {
        roller.set(-RobotConstants.get().intake2020RollerBackwardSpeed());
      }

    }

    /**
    * turn off roller motor so that it does not grab balls
    */
    public void rollerStop() {
      roller.set(0);


    }
    @Override
    public void periodic() {

    }

}
