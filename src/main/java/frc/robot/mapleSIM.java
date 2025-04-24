package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

public class mapleSIM {
  public static SimulatedArena arena = SimulatedArena.getInstance();
  final DriveTrainSimulationConfig driveTrainSimulationConfig =
      DriveTrainSimulationConfig.Default()
          .withGyro(COTS.ofPigeon2())
          .withSwerveModule(
              new SwerveModuleSimulationConfig(
                  DCMotor.getKrakenX60Foc(1),
                  DCMotor.getNeo550(1),
                  6.75,
                  12.8,
                  Volts.of(0.1),
                  Volts.of(0.1),
                  Inches.of(2),
                  KilogramSquareMeters.of(7.2777),
                  1.2))
          .withTrackLengthTrackWidth(Meters.of(0.3), Meters.of(0.29))
          .withBumperSize(Meters.of(0.95), Meters.of(0.95));

  public mapleSIM() {
    SwerveDriveSimulation swereDriveSimulation =
        new SwerveDriveSimulation(driveTrainSimulationConfig, new Pose2d(3, 3, new Rotation2d()));
    arena.addDriveTrainSimulation(swereDriveSimulation);
  }
}
