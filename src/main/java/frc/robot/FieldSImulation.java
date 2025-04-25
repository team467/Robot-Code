package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.crescendo2024.CrescendoNoteOnField;
import org.littletonrobotics.junction.AutoLogOutput;

public class FieldSImulation {
  @AutoLogOutput private Pose2d simPose;
  @AutoLogOutput private Pose3d[] gamePieces;

  public FieldSImulation(SwerveDriveSimulation swerveDriveSimulation) {
    this.simPose = swerveDriveSimulation.getSimulatedDriveTrainPose();
    this.gamePieces = SimulatedArena.getInstance().getGamePiecesArrayByType("Coral");
  }

  public void addCoral() {
    SimulatedArena.getInstance().addGamePiece(new CrescendoNoteOnField(new Translation2d(2, 2)));
    this.gamePieces = SimulatedArena.getInstance().getGamePiecesArrayByType("Note");
  }
}
