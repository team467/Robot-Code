package frc.lib.autocheck;

import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.Pigeon2;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.autocheck.selfcheck.*;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

/**
 * The FaultReporter class is responsible for detecting and reporting faults in the robot
 * subsystems. It provides methods for registering system checks, adding faults, clearing faults,
 * and checking for faults. The class maintains a list of subsystem faults and a list of
 * self-checking hardware components. The system status and faults are periodically published to the
 * logger for monitoring.
 */
public class FaultReporter {
  public enum SystemStatus {
    OK,
    WARNING,
    ERROR
  }

  private static class SubsystemFaults {
    private List<SubsystemFault> faults = new ArrayList<>();
    private List<SelfChecking> hardware = new ArrayList<>();
  }

  private static FaultReporter instance = null;

  private static final String CHECK_RAN = "/CheckRan";
  private static final String SYSTEM_STATUS = "SystemStatus/";

  private final Map<String, SubsystemFaults> subsystemsFaults = new HashMap<>();
  private final boolean checkErrors;

  private FaultReporter() {
    this.checkErrors = RobotBase.isReal();
    setupCallbacks();
  }

  public static FaultReporter getInstance() {
    if (instance == null) {
      instance = new FaultReporter();
    }
    return instance;
  }

  /**
   * Registers a system check command for a specific subsystem.
   *
   * @param subsystemName the name of the subsystem
   * @param systemCheckCommand the system check command to register
   * @return the wrapped system check command
   */
  public CommandBase registerSystemCheck(String subsystemName, CommandBase systemCheckCommand) {
    String statusTable = SYSTEM_STATUS + subsystemName;
    SubsystemFaults subsystemFaults =
        subsystemsFaults.getOrDefault(subsystemName, new SubsystemFaults());

    CommandBase wrappedSystemCheckCommand =
        wrapSystemCheckCommand(subsystemName, systemCheckCommand);
    wrappedSystemCheckCommand.setName(subsystemName + "Check");
    SmartDashboard.putData(statusTable + "/SystemCheck", wrappedSystemCheckCommand);
    Logger.getInstance().recordOutput(statusTable + CHECK_RAN, false);

    subsystemsFaults.put(subsystemName, subsystemFaults);

    return wrappedSystemCheckCommand;
  }

  /**
   * Wraps a system check command with additional commands to handle logging and status publishing.
   *
   * @param subsystemName the name of the subsystem
   * @param systemCheckCommand the system check command to wrap
   * @return the wrapped system check command
   */
  private CommandBase wrapSystemCheckCommand(String subsystemName, CommandBase systemCheckCommand) {
    String statusTable = SYSTEM_STATUS + subsystemName;
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              Logger.getInstance().recordOutput(statusTable + CHECK_RAN, false);
              clearFaults(subsystemName);
              publishStatus();
            }),
        systemCheckCommand,
        Commands.runOnce(
            () -> {
              publishStatus();
              Logger.getInstance().recordOutput(statusTable + CHECK_RAN, true);
            }));
  }

  /** Set up callbacks for checking faults and publishing status on a repeating schedule. */
  private void setupCallbacks() {
    CommandScheduler.getInstance()
        .schedule(
            Commands.repeatingSequence(
                    Commands.runOnce(this::checkForFaults), Commands.waitSeconds(0.25))
                .ignoringDisable(true));

    CommandScheduler.getInstance()
        .schedule(
            Commands.repeatingSequence(
                    Commands.runOnce(this::publishStatus), Commands.waitSeconds(1.0))
                .ignoringDisable(true));
  }

  /**
   * Publishes the status of each subsystem by recording output in the logger. It records the
   * status, system OK indicator, faults, and last fault for each subsystem.
   */
  private void publishStatus() {
    for (Map.Entry<String, SubsystemFaults> entry : subsystemsFaults.entrySet()) {
      String subsystemName = entry.getKey();
      SubsystemFaults subsystemFaults = entry.getValue();

      SystemStatus status = getSystemStatus(subsystemFaults.faults);

      String statusTable = SYSTEM_STATUS + subsystemName;
      Logger.getInstance().recordOutput(statusTable + "/Status", status.name());
      Logger.getInstance().recordOutput(statusTable + "/SystemOK", status == SystemStatus.OK);

      String[] faultStrings = new String[subsystemFaults.faults.size()];
      for (int i = 0; i < subsystemFaults.faults.size(); i++) {
        SubsystemFault fault = subsystemFaults.faults.get(i);
        faultStrings[i] = String.format("[%.2f] %s", fault.timestamp, fault.description);
      }
      Logger.getInstance().recordOutput(statusTable + "/Faults", faultStrings);

      if (faultStrings.length > 0) {
        Logger.getInstance()
            .recordOutput(statusTable + "/LastFault", faultStrings[faultStrings.length - 1]);
      } else {
        Logger.getInstance().recordOutput(statusTable + "/LastFault", "");
      }
    }
  }

  /**
   * Adds a fault to the specified subsystem's fault list.
   *
   * <p>If the fault is already present in the list, it will not be added again.
   *
   * @param subsystemName the name of the subsystem
   * @param fault the fault to add
   */
  public void addFault(String subsystemName, SubsystemFault fault) {
    List<SubsystemFault> subsystemFaults = subsystemsFaults.get(subsystemName).faults;
    if (!subsystemFaults.contains(fault)) {
      subsystemFaults.add(fault);
    }
  }

  /**
   * Adds a fault to the specified subsystem's fault list.
   *
   * <p>If the fault is already present in the list, it will not be added again.
   *
   * @param subsystemName the name of the subsystem
   * @param description the description of the fault
   * @param isWarning true if the fault is a warning, false if it is an error
   */
  public void addFault(String subsystemName, String description, boolean isWarning) {
    this.addFault(subsystemName, new SubsystemFault(description, isWarning));
  }

  /**
   * Adds a fault to the specified subsystem's fault list.
   *
   * <p>If the fault is already present in the list, it will not be added again.
   *
   * @param subsystemName the name of the subsystem
   * @param description the description of the fault
   * @param isWarning true if the fault is a warning, false if it is an error
   * @param sticky true if the fault should be sticky and not cleared automatically, false otherwise
   */
  public void addFault(
      String subsystemName, String description, boolean isWarning, boolean sticky) {
    this.addFault(subsystemName, new SubsystemFault(description, isWarning, sticky));
  }

  /**
   * Adds a fault to the specified subsystem's fault list.
   *
   * @param subsystemName the name of the subsystem
   * @param description the description of the fault
   */
  public void addFault(String subsystemName, String description) {
    this.addFault(subsystemName, description, false);
  }

  /**
   * Gets the list of faults for the specified subsystem.
   *
   * @param subsystemName the name of the subsystem
   * @return a list of {@link SubsystemFault} objects representing the faults in the subsystem
   */
  public List<SubsystemFault> getFaults(String subsystemName) {
    return subsystemsFaults.get(subsystemName).faults;
  }

  /**
   * Clears all the faults for the specified subsystem.
   *
   * @param subsystemName the name of the subsystem
   */
  public void clearFaults(String subsystemName) {
    subsystemsFaults.get(subsystemName).faults.clear();
  }

  /**
   * Calculates and returns the system status based on the list of subsystem faults.
   *
   * @param subsystemFaults the list of subsystem faults
   * @return the calculated system status
   */
  private SystemStatus getSystemStatus(List<SubsystemFault> subsystemFaults) {
    SystemStatus worstStatus = SystemStatus.OK;

    for (SubsystemFault f : subsystemFaults) {
      if (f.sticky || f.timestamp > Timer.getFPGATimestamp() - 10) {
        if (f.isWarning) {
          if (worstStatus != SystemStatus.ERROR) {
            worstStatus = SystemStatus.WARNING;
          }
        } else {
          worstStatus = SystemStatus.ERROR;
        }
      }
    }
    return worstStatus;
  }

  /**
   * Registers a hardware component in the specified subsystem.
   *
   * @param subsystemName the name of the subsystem
   * @param label the label of the hardware component
   * @param phoenixMotor the BaseMotorController instance representing the hardware component
   */
  public void registerHardware(
      String subsystemName, String label, BaseMotorController phoenixMotor) {
    SubsystemFaults subsystemFaults =
        subsystemsFaults.getOrDefault(subsystemName, new SubsystemFaults());
    subsystemFaults.hardware.add(new SelfCheckingPhoenixMotor(label, phoenixMotor));
    subsystemsFaults.put(subsystemName, subsystemFaults);
  }

  /**
   * Registers a hardware component in the specified subsystem.
   *
   * @param subsystemName the name of the subsystem
   * @param label the label of the hardware component
   * @param pwmMotor the PWMMotorController instance representing the hardware component
   */
  public void registerHardware(String subsystemName, String label, PWMMotorController pwmMotor) {
    SubsystemFaults subsystemFaults =
        subsystemsFaults.getOrDefault(subsystemName, new SubsystemFaults());
    subsystemFaults.hardware.add(new SelfCheckingPWMMotor(label, pwmMotor));
    subsystemsFaults.put(subsystemName, subsystemFaults);
  }

  /**
   * Registers a hardware component in the specified subsystem.
   *
   * @param subsystemName the name of the subsystem
   * @param label the label of the hardware component
   * @param spark the CANSparkMax instance representing the hardware component
   */
  public void registerHardware(String subsystemName, String label, CANSparkMax spark) {
    SubsystemFaults subsystemFaults =
        subsystemsFaults.getOrDefault(subsystemName, new SubsystemFaults());
    subsystemFaults.hardware.add(new SelfCheckingSparkMax(label, spark));
    subsystemsFaults.put(subsystemName, subsystemFaults);
  }

  /**
   * Registers a hardware component in the specified subsystem.
   *
   * @param subsystemName the name of the subsystem
   * @param label the label of the hardware component
   * @param pigeon2 the Pigeon2 instance representing the hardware component
   */
  public void registerHardware(String subsystemName, String label, Pigeon2 pigeon2) {
    SubsystemFaults subsystemFaults =
        subsystemsFaults.getOrDefault(subsystemName, new SubsystemFaults());
    subsystemFaults.hardware.add(new SelfCheckingPigeon2(label, pigeon2));
    subsystemsFaults.put(subsystemName, subsystemFaults);
  }

  /**
   * Registers a hardware component in the specified subsystem.
   *
   * @param subsystemName the name of the subsystem
   * @param label the label of the hardware component
   * @param canCoder the CANCoder instance representing the hardware component
   */
  public void registerHardware(String subsystemName, String label, CANCoder canCoder) {
    SubsystemFaults subsystemFaults =
        subsystemsFaults.getOrDefault(subsystemName, new SubsystemFaults());
    subsystemFaults.hardware.add(new SelfCheckingCANCoder(label, canCoder));
    subsystemsFaults.put(subsystemName, subsystemFaults);
  }

  /**
   * Checks for any faults in the registered hardware components of all subsystems.
   *
   * <p>If checkErrors is set to true, it iterates through each subsystem and its registered
   * hardware components, and calls the checkForFaults() method on each registered hardware
   * component.
   *
   * <p>Any detected faults are added to the list of faults for the respective subsystem.
   *
   * <p>Note: The checkErrors flag must be set to true before calling this method, otherwise no
   * faults will be checked.
   */
  private void checkForFaults() {
    if (checkErrors) {
      for (Map.Entry<String, SubsystemFaults> entry : subsystemsFaults.entrySet()) {
        String subsystemName = entry.getKey();
        SubsystemFaults subsystemFaults = entry.getValue();
        for (SelfChecking device : subsystemFaults.hardware) {
          for (SubsystemFault fault : device.checkForFaults()) {
            addFault(subsystemName, fault);
          }
        }
      }
    }
  }
}
