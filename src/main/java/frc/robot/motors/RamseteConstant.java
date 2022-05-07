package frc.robot.motors;

import edu.wpi.first.math.controller.RamseteController;

public class RamseteConstant {
    private final double kB;
    private final double kZeta;

    public RamseteConstant(double kB, double kZeta) {
        this.kB = kB;
        this.kZeta = kZeta;
    }

    public RamseteConstant() {
        this(2.0, 0.7);
    }

    public double getkB() {
        return kB;
    }

    public double getkZeta() {
        return kZeta;
    }

    public RamseteController getController() {
        return new RamseteController(kB, kZeta);
    }
}
