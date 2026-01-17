package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;


  public interface IntakeIO {
    @AutoLog
    class IntakeIOInputs {
      public double percentOutput = 0.0;
      public double volts = 0.0;
      public double amps = 0.0;
      public boolean isExtended = false;
    }

    default void updateInputs(IntakeIOInputs inputs){}

    default void setPercent(double percent){}

    default void setVoltage(double volts){}

    default void stop(){}

    default boolean isHopperExtended(){return false;}



  }

