package frc.lib.characterization;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;

/**
 * A tool that creates SysId tests compatible with AdvantageKit.
 *
 * <p>Be sure to follow <a
 * href="https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/docs/SYSID.md#loading-data">these
 * instructions</a> to load the log file into the SysId tool.
 *
 * @see SysIdRoutine
 * @see <a
 *     href="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/index.html">SysId
 *     Documentation</a>
 */
public class SysIdFactory {
  private final SysIdRoutine sysId;

  /**
   * Creates a SysIdFactory object.
   *
   * <p>Defaults to a ramp rate of 1 volt/sec, a step voltage of 7 volts, and a timeout of 10
   * seconds.
   *
   * @param subsystem The subsystem containing the motor(s) that is (or are) being characterized.
   *     Will be declared as a requirement for the returned test commands.
   * @param driveVoltage Sends the SysId-specified drive signal to the mechanism motors during test
   *     routines.
   */
  public SysIdFactory(Subsystem subsystem, Consumer<Measure<Voltage>> driveVoltage) {
    this(subsystem, driveVoltage, new SysIdRoutine.Config());
  }

  /**
   * Creates a SysIdFactory object.
   *
   * @param subsystem The subsystem containing the motor(s) that is (or are) being characterized.
   *     Will be declared as a requirement for the returned test commands.
   * @param driveVoltage Sends the SysId-specified drive signal to the mechanism motors during test
   *     routines.
   * @param config Hardware-independent parameters for the SysId routine, for advanced
   *     configurations. The 'recordState' variable will be overwritten with AdvantageKit compatible
   *     code.
   */
  public SysIdFactory(
      Subsystem subsystem, Consumer<Measure<Voltage>> driveVoltage, SysIdRoutine.Config config) {
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                config.m_rampRate,
                config.m_stepVoltage,
                config.m_timeout,
                state -> Logger.recordOutput(subsystem.getName() + "/SysIdState", state)),
            new SysIdRoutine.Mechanism(
                driveVoltage,
                null, // No log consumer, since data is recorded by AdvantageKit
                subsystem));
  }

  /**
   * Returns a command to run a quasistatic test in the specified direction.
   *
   * <p>In this test, the mechanism is gradually sped up such that the voltage corresponding to
   * acceleration is negligible (hence, “as if static”).
   *
   * <p>The command will call the `driveVoltage` callbacks supplied at routine construction once per
   * iteration. Upon command end or interruption, the `driveVoltage` callback is called with a value
   * of 0 volts.
   *
   * @param direction The direction in which to run the test.
   * @return A command to run the test.
   */
  public Command quasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /**
   * Returns a command to run a dynamic test in the specified direction.
   *
   * <p>In this test, a constant ‘step voltage’ is given to the mechanism, so that the behavior
   * while accelerating can be determined.
   *
   * <p>The command will call the `driveVoltage` callbacks supplied at routine construction once per
   * iteration. Upon command end or interruption, the `driveVoltage` callback is called with a value
   * of 0 volts.
   *
   * @param direction The direction in which to run the test.
   * @return A command to run the dynamic test.
   */
  public Command dynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }
}
