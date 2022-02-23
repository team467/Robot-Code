package frc.robot.motors;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;

public class SimpleFeedforwardConstant {
    private final double kS;
    private final double kV;
    private final double kA;

    public SimpleFeedforwardConstant(double kS, double kV, double kA) {
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
    }

    public SimpleFeedforwardConstant(double kS, double kV) {
        this.kS = kS;
        this.kV = kV;
        this.kA = 0;
    }

    public double getkS() {
        return kS;
    }

    public double getkV() {
        return kV;
    }

    public double getkA() {
        return kA;
    }

    public SimpleMotorFeedforward getFeedforward() {
        return new SimpleMotorFeedforward(kS, kV, kA);
    }

    public LinearSystem<N1, N1, N1> getVelocityPlant() {
        return LinearSystemId.identifyVelocitySystem(kV, kA);
    }

    public LinearSystem<N2, N1, N1> getPositionPlant() {
        return LinearSystemId.identifyPositionSystem(kV, kA);
    }
}
