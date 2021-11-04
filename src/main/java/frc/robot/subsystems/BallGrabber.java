package frc.robot.subsystems;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.drive.MotorSpeedController;
import frc.robot.drive.TalonController;

public class BallGrabber extends SubsystemBase {

    /**
    * TODO:
    * STAGE 1: Use 1 motor to switch from up to down position
    * STAGE 2: Enable both motor 2(regular wheels that grab balls from ground) and 3(rubbery thingy that moves ball from roller section to indexer section) to begin grabbing balls
    * STAGE 3: Use the indexer motor to bring to shoot area
    */

  //! not functional: private TalonSRX grabberPositionChanger = new TalonSRX(Constants.DRIVE_MOTOR_GRABBER_POSITION_SETTER);
  //! not functional: private TalonSRX grabber = new TalonSRX(Constants.DRIVE_MOTOR_GRABBER);

    public BallGrabber() {
      super();
    }

    //stage 1

    public void lowerGrabber() {

      //TODO: configure the lowering of grabber to allow it to grab balls.
      /*
      set motor to speed to lower grabber
      wait some time which is the optimal angle
      turn off motor
      */
      //? how much speed and for how long does it lower?

      /** PSEUDOCODE 
       * 
      */

    }
    
    public void raiseGrabber() {

      //TODO: configure the raising of grabber to allow it to be placed at the start of a competition.
      /*
      set motor to speed to raise grabber
      wait some time which is the optmial angle
      turn off motor
      */
      //? how much speed and for how long does it rise?

      /** PSEUDOCODE 
       * 
      */

    }

    //stage 2
    public void enableGrabber() {

      //TODO: set grabber speed so that it can grab balls and place them in storage
      /*
      set grabber motor to regular speed
      set rubber motor to regular speed
      */
      //! not functional: grabber.set(Constants.DRIVE_MOTOR_GRABBER_SPEED);
      //? how do you configure a TalonSRX motor to move forward and backwards?

      /** PSEUDOCODE 
       * 
      */
      
    }

    public void reverseGrabber() {

      //TODO: set grabber speed to the reverse of DRIVE_MOTOR_GRABBER_SPEED so that it can unstuck any balls.
      /*
      set grabber motor to negative of regular speed
      set rubber motor to negative of regular speed
      */
      //! not functional: grabber.set(-Constants.DRIVE_MOTOR_GRABBER_SPEED);
      //? how do you configure a TalonSRX motor to move forward and backwards?

      /** PSEUDOCODE 
       * 
      */
      
    }

    public void disableGrabber() {

      //TODO: set grabber speed to 0 so that it does not grab balls
      /*
      turn off grabber motor
      turn off rubber motor
      */
      //! not functional: grabber.set(0);
      //? how do you configure a TalonSRX motor to move forward and backwards?

      /** PSEUDOCODE 
       * 
      */

    }

}

