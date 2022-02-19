package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter2022 extends SubsystemBase {
  public final Indexer2022 indexer2022;
  public final LlamaNeck2022 llamaNeck2022;
  public final Spitter2022 spitter2022;

  public Shooter2022(Indexer2022 indexer2022, LlamaNeck2022 llamaNeck2022, Spitter2022 spitter2022) {
    super();

    this.indexer2022 = indexer2022;
    this.llamaNeck2022 = llamaNeck2022;
    this.spitter2022 = spitter2022;
  }
}
