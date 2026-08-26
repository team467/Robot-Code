package frc.robot.sim;

import static org.junit.jupiter.api.Assertions.*;

import frc.robot.subsystems.climber.ClimberIO.ClimberIOInputs;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.indexer.IndexerIO.IndexerIOInputs;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.intake.extend.IntakeExtendIO.IntakeExtendIOInputs;
import frc.robot.subsystems.intake.extend.IntakeExtendIOSim;
import frc.robot.subsystems.intake.rollers.IntakeRollersIO.IntakeRollersIOInputs;
import frc.robot.subsystems.intake.rollers.IntakeRollersIOSim;
import frc.robot.subsystems.magicCarpet.MagicCarpetIO.MagicCarpetIOInputs;
import frc.robot.subsystems.magicCarpet.MagicCarpetIOSim;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;
import frc.robot.subsystems.shooter.ShooterIOSim;
import org.junit.jupiter.api.Test;

public class SubsystemSimTest {

  @Test
  public void testShooterIOSim() {
    ShooterIOSim shooterSim = new ShooterIOSim();
    ShooterIOInputs inputs = new ShooterIOInputs();

    shooterSim.setVoltage(12.0);
    for (int i = 0; i < 50; i++) {
      shooterSim.updateInputs(inputs);
    }

    assertTrue(
        inputs.shooterWheelVelocityRadPerSec > 10.0,
        "Shooter wheel should accelerate under voltage");
    assertEquals(12.0, inputs.bottomMotorAppliedVolts);
    assertTrue(inputs.totalAmps >= 0.0);

    shooterSim.stop();
    shooterSim.updateInputs(inputs);
    assertEquals(0.0, inputs.bottomMotorAppliedVolts);
  }

  @Test
  public void testIndexerIOSim() {
    IndexerIOSim indexerSim = new IndexerIOSim();
    IndexerIOInputs inputs = new IndexerIOInputs();

    indexerSim.setVoltage(8.0);
    indexerSim.updateInputs(inputs);

    assertEquals(8.0, inputs.feedUpVolts);
    assertTrue(inputs.feedUpAmps > 0);

    // Test with ball in robot
    BallSimulator.getInstance().setBallsInRobot(1);
    indexerSim.updateInputs(inputs);
    assertTrue(inputs.ballAtLeftSwitch);
    assertTrue(inputs.ballAtRightSwitch);

    BallSimulator.getInstance().setBallsInRobot(0);
    indexerSim.updateInputs(inputs);
    assertFalse(inputs.ballAtLeftSwitch);
    assertFalse(inputs.ballAtRightSwitch);
  }

  @Test
  public void testMagicCarpetIOSim() {
    MagicCarpetIOSim carpetSim = new MagicCarpetIOSim();
    MagicCarpetIOInputs inputs = new MagicCarpetIOInputs();

    carpetSim.setSpeed(0.8);
    carpetSim.updateInputs(inputs);

    assertEquals(0.8 * 12.0, inputs.appliedVolts, 0.01);
    assertTrue(inputs.motorVelocity > 0);

    carpetSim.stop();
    carpetSim.updateInputs(inputs);
    assertEquals(0.0, inputs.appliedVolts);
  }

  @Test
  public void testIntakeRollersIOSim() {
    IntakeRollersIOSim rollersSim = new IntakeRollersIOSim();
    IntakeRollersIOInputs inputs = new IntakeRollersIOInputs();

    rollersSim.setVoltageIntake(12.0);
    rollersSim.updateInputs(inputs);

    assertEquals(12.0, inputs.intakeVolts);
    assertTrue(inputs.intakeRPM > 0);

    rollersSim.stop();
    rollersSim.updateInputs(inputs);
    assertEquals(0.0, inputs.intakeVolts);
  }

  @Test
  public void testIntakeExtendIOSim() {
    IntakeExtendIOSim extendSim = new IntakeExtendIOSim();
    IntakeExtendIOInputs inputs = new IntakeExtendIOInputs();

    extendSim.goToPos(-1.34);
    for (int i = 0; i < 50; i++) {
      extendSim.updateInputs(inputs);
    }

    assertTrue(inputs.getExtendPos < 0.0, "Intake extend should move toward extended position");
  }

  @Test
  public void testClimberIOSim() {
    ClimberIOSim climberSim = new ClimberIOSim();
    ClimberIOInputs inputs = new ClimberIOInputs();

    climberSim.setPercent(0.5);
    for (int i = 0; i < 20; i++) {
      climberSim.updateInputs(inputs);
    }

    assertTrue(
        inputs.positionDegrees > 0.0, "Climber should rotate when percent output is applied");
    assertTrue(inputs.isCalibrated);
  }
}
