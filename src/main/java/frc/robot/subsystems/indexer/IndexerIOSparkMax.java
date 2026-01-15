package frc.robot.subsystems.indexer;

import static frc.robot.subsystems.indexer.IndexerConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.AnalogTrigger;
import edu.wpi.first.wpilibj.DigitalInput;

public class IndexerIOSparkMax implements IndexerIO {

    private final SparkMax motor;
    private final DigitalInput limitSwitch;

    public IndexerIOSparkMax() {
        motor = new SparkMax(INDEXER_MOTOR_ID, MotorType.kBrushed);

        var config = new SparkMaxConfig();
        config.inverted(true)
                .idleMode(IdleMode.kBrake)
                .voltageCompensation(12)
                .smartCurrentLimit(30);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        limitSwitch = new DigitalInput(0);
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.percentOutput = motor.get();
        inputs.volts = motor.getBusVoltage() * motor.getAppliedOutput();
        inputs.amps = motor.getOutputCurrent();
        inputs.ballAtSwitch = isSwitchPressed()
        ;
    }

    @Override
    public void setPercent(double percent) {
        motor.set(percent);
    }

    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }
    @Override
    public void stop() {
        motor.set(0);
    }

    public boolean isSwitchPressed() {
        return limitSwitch.get();
    }
}

