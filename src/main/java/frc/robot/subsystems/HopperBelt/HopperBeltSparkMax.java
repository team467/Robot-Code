// Controls the motor
// Reads motor data every 20 ms
// Implements the methods defined in HopperBeltIO
package frc.robot.subsystems.HopperBelt;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;

public class HopperBeltSparkMax implements HopperBeltIO {

  private final SparkMax motor; // object controllin motor
  private final RelativeEncoder encoder; // reads motor speed

  public HopperBeltSparkMax() {
    // Create Spark MAX for NEO 1.1 (brushless)
    motor =
        new SparkMax(
            HopperBeltConstants.MOTOR_ID,
            MotorType.kBrushless); // creats the sparkMAX object and tells what motor this is

    // Configuration object, so we wont have to call many settlers and apply all at once
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .inverted(HopperBeltConstants.MOTOR_INVERTED)
        /**
         * to make sure that a positive speed makes the belt move forward physically. Since it’s set
         * to true right now, the motor output is flipped so that forward code still results in
         * forward belt motion. *
         */
        .idleMode(IdleMode.kBrake) // stops motor quickly when set to 0
        .smartCurrentLimit(HopperBeltConstants.CURRENT_LIMIT); // sets maximum current amps

    // Apply configuration to motor and save to controller
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // to measure speed
    encoder = motor.getEncoder();
  }

  @Override
  public void updateInputs(HopperBeltIOInputs inputs) {
    // Called every 20 ms by subsystem periodic
    inputs.appliedOutput = motor.getAppliedOutput(); // 0.0 -> 1.0
    inputs.motorCurrent = motor.getOutputCurrent(); // Amps
    inputs.motorVelocity = encoder.getVelocity(); // RPM
  }

  @Override
  public void setSpeed(double speed) {

    motor.set(MathUtil.clamp(speed, 0.0, 1.0)); // so the is between 0 and 1, 1==100%
  }

  @Override // implements the stop method from interface, and sets the speed to 0, meaning it
  // immidately stops
  public void stop() {
    motor.set(0.0);
  }
}
