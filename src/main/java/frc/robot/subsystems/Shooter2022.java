package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.NetworkButton;
import frc.robot.commands.Shooter2022IdleCMD;
import frc.robot.commands.Shooter2022ShootSpeedCMD;
import frc.robot.tuning.SubsystemTuner;

public class Shooter2022 extends SubsystemTuner {
  public final Indexer2022 indexer2022;
  public final LlamaNeck2022 llamaNeck2022;
  public final Spitter2022 spitter2022;

  public Shooter2022(Indexer2022 indexer2022, LlamaNeck2022 llamaNeck2022, Spitter2022 spitter2022) {
    super();

    this.indexer2022 = indexer2022;
    this.llamaNeck2022 = llamaNeck2022;
    this.spitter2022 = spitter2022;
  }

  @Override
  public void initializeTunerNetworkTables(ShuffleboardTab tab) {
    addEntry("speed", tab.add("Flywheel Speed", 0).withWidget(BuiltInWidgets.kNumberSlider).withSize(2, 1).withPosition(4, 1).withProperties(Map.of("min", 0, "max", 1)).getEntry());
    addEntry("run", tab.add("Run", false).withWidget(BuiltInWidgets.kToggleButton).withSize(2, 1).withPosition(4, 3).getEntry());   
  }

  @Override
  public void initializeTuner() {
    this.setDefaultCommand(new Shooter2022IdleCMD(this));

    getEntry("speed").setDouble(0);
    getEntry("run").setBoolean(false);

    new NetworkButton(getEntry("run")).whileActiveContinuous(
    new Shooter2022ShootSpeedCMD(this, 
        () -> getEntry("speed").getDouble(0)
    ));
  }
}
