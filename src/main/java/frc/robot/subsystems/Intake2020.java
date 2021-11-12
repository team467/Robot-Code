package frc.robot.subsystems;
/*
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
*/
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.drive.TalonController;

public class Intake2020 extends SubsystemBase {

    TalonController arm = new TalonController(Constants.INTAKE2020_ARM_MOTOR_ID);
    TalonController roller = new TalonController(Constants.INTAKE2020_ROLLER_MOTOR_ID);
    
    Boolean down = false;

    public Intake2020() {
      super();
    }

    //stage 1

      /**
       * Lowers the arm so that it can pick up balls.
      */
      public void lowerArm() {

      arm.set(Constants.INTAKE2020_ARM_DOWN_SPEED);
      down = true;

    }

    /**
    * Raises the arm so that it's in the robot permeter for game start or when playing defense.
    */
    public void raiseGrabber() {

      this.stopRoller();
      arm.set(Constants.INTAKE2020_ARM_UP_SPEED);
      down = false;

    }

    //stage 2
    /**
    * set roller motor to regular speed so that it can pick up balls and place them into indexing
    */
    public void forwardRoller() {

      if(down) {
        roller.set(Constants.INTAKE2020_ROLLER_FORWARD_SPEED);  
      }  

    }

    /**
    * set roller motor to reverse so that it can unstuck any balls
    */
    public void reverseRoller() {

      if(down) {
        roller.set(-Constants.INTAKE2020_ROLLER_BACKWARD_SPEED);
      }

    }

    /**
    * turn off roller motor so that it does not grab balls
    */
    public void stopRoller() {
      
      roller.set(0);

    }

}

    