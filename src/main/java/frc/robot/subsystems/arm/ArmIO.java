package frc.robot.subsystems.arm;

public interface ArmIO {
 
    class ArmIOInputs {
        public double armUpPosition = 0.0;
        public double armDownPosition = 0.0;
        public double armExtendPosition = 0.0;
        public double armRetractPosition = 0.0;
        public double armVeloctiy = 0.0;
    }

    default void setArmPercent (double percent) {}
    default void setArmVoltage(double volts) {}

}
