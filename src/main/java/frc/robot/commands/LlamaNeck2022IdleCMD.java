package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.logging.RobotLogManager;
import frc.robot.subsystems.LlamaNeck2022;
import org.apache.logging.log4j.Logger;

public class LlamaNeck2022IdleCMD extends CommandBase {
  private static final Logger LOGGER =
      RobotLogManager.getMainLogger(LlamaNeck2022IdleCMD.class.getName());
  private final LlamaNeck2022 llamaNeck;

  public LlamaNeck2022IdleCMD(LlamaNeck2022 llamaNeck) {
    this.llamaNeck = llamaNeck;
    addRequirements(llamaNeck);
  }

  @Override
  public void initialize() {
    LOGGER.debug("Setting Llama Neck idle");
  }

  @Override
  public void execute() {
    llamaNeck.idle();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
