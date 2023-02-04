package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;

public class Arm2023ScoreMidNodeCMD extends CommandBase {
    private final Arm arm;

    public Arm2023ScoreMidNodeCMD(Arm arm) {
      this.arm = arm;
  
      addRequirements(arm);
    }
  
    @Override
    public void execute() {
        arm.setExtendSetpoint(0.6);
    }
  
    @Override
    public void end(boolean interrupted) {}
  
    @Override
    public boolean isFinished() {
      return false;
    }
  }
  






