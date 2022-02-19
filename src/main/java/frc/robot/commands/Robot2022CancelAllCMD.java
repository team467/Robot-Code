package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.logging.RobotLogManager;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer2022;
import frc.robot.subsystems.LlamaNeck2022;
import frc.robot.subsystems.Spitter2022;
import org.apache.logging.log4j.Logger;

public class Robot2022CancelAllCMD extends InstantCommand {

  private static final Logger LOGGER =
      RobotLogManager.getMainLogger(Robot2022CancelAllCMD.class.getName());

  private final Indexer2022 indexer;
  private final LlamaNeck2022 llamaNeck;
  private final Spitter2022 spitter;
  private final Drivetrain drivetrain;

  public Robot2022CancelAllCMD(
      Indexer2022 indexer, LlamaNeck2022 llamaNeck, Spitter2022 spitter, Drivetrain drivetrain) {
    this.indexer = indexer;
    this.llamaNeck = llamaNeck;
    this.spitter = spitter;
    this.drivetrain = drivetrain;
    addRequirements(indexer, llamaNeck, spitter, drivetrain);
  }

  @Override
  public void initialize() {
    LOGGER.debug("Panic button pressed, cancel commands!");
    indexer.getCurrentCommand().cancel();
    llamaNeck.getCurrentCommand().cancel();
    spitter.getCurrentCommand().cancel();
    drivetrain.getCurrentCommand().cancel();
  }

  @Override
  public void end(boolean interrupted) {}
}
