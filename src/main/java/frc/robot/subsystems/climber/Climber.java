package frc.robot.subsystems.climber;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;


public class Climber extends SubsystemBase {


 private final ClimberIO climberIO;
 private final ClimberIOInputsAutoLogged climberIOInputs = new ClimberIOInputsAutoLogged();


 /**
  * ClimberIO object, gets inputs for ClimberIO object
  *
  * @param climberIO
  */
 public Climber(ClimberIO climberIO) {
   super();


   this.climberIO = climberIO;
   climberIO.resetPosition();
 }


 public void periodic() {
   climberIO.updateInputs(climberIOInputs);
   Logger.processInputs("Climber", climberIOInputs);
   RobotState.getInstance().climberRatchet = climberIOInputs.ratchetLocked;
   if (getLimitSwitch()) {
     climberIO.resetPosition();
   }
 }


 /**
  * Command to raise or lower the climber arms If percentOutput is negative, the climber will lower
  * If percentOutput is positive, the climber will raise If percentOutput is 0, the climber will
  * stop If the ratchet is locked, the climber will not move. It does nothing before the command
  * and then checks if the ratchet is locked. Ends the command if the ratchet is locked.
  *
  * @param percentOutput takes a number from -1 to 1.
  * @return no return
  */
 public Command raiseOrLower(double speed) {
   return Commands.run(
           () -> {
             Logger.recordOutput("Climber/OutputDesired", speed);
             climberIO.setSpeed(speed);
           },
           this)
       .onlyWhile(() -> !climberIOInputs.ratchetLocked)
       .beforeStarting( // This is for testing the motors before. Might be taken out.
           Commands.none()
               .alongWith(
                   Commands.runOnce(
                       () -> {
                         RobotState.getInstance().climberUp = speed > 0;
                         RobotState.getInstance().climberDown = speed < 0;
                       })))
       .onlyWhile(() -> !climberIOInputs.ratchetLocked)
       .finallyDo(
           () -> {
             RobotState.getInstance().climberUp = false;
             RobotState.getInstance().climberDown = false;
           });
 }
 /**
  * Command to disable the climber
  *
  * @return no return
  */
 public Command setRatchet(boolean locked) {
   return Commands.runOnce(
       () -> {
         climberIO.setRatchetLocked(locked);
       },
       this);
 }


 //TODO: Review design to see if we need to move backwards to release ratchet.


 public boolean getRatchet() {
   return climberIOInputs.ratchetLocked;
 }


 public boolean getLimitSwitch() {
   return true;
 }
}

