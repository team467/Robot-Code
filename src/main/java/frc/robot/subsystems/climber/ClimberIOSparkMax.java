package frc.robot.subsystems.climber;


import static frc.lib.utils.SparkUtil.tryUntilOk;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.Relay;




public class ClimberIOSparkMax implements ClimberIO {


 private final SparkFlex climberLeader;
 private final RelativeEncoder climberLeaderEncoder;
 private final SparkFlex climberFollower;
 private final Relay climberRatchet;
 private SparkLimitSwitch limitSwitch;
 private final int CLIMBER_LEADER_ID = 1;
 private final int CLIMBER_FOLLOWER_ID = 2;


 public ClimberIOSparkMax() {
   // Motors and Encoders
   climberLeader = new SparkFlex(CLIMBER_LEADER_ID, MotorType.kBrushless);
   climberLeaderEncoder = climberLeader.getEncoder();
   var ClimberLeaderConfig = new SparkFlexConfig();
   ClimberLeaderConfig.inverted(true).idleMode(IdleMode.kBrake).voltageCompensation(12).smartCurrentLimit(40);
   ClimberLeaderConfig.softLimit.forwardSoftLimit(10).forwardSoftLimitEnabled(true).reverseSoftLimitEnabled(false);
   ClimberLeaderConfig.encoder.positionConversionFactor(ClimberConstants.ROTS_TO_METERS);


   climberFollower = new SparkFlex(CLIMBER_FOLLOWER_ID, MotorType.kBrushless);
   var ClimberFollowerConfig = new SparkFlexConfig();
   ClimberFollowerConfig.follow(1);


   tryUntilOk(
       climberLeader,
       5,
       () ->
           climberLeader.configure(
               ClimberLeaderConfig,
               SparkBase.ResetMode.kResetSafeParameters,
               PersistMode.kPersistParameters));
   tryUntilOk(climberLeader, 5, () -> climberLeaderEncoder.setPosition(0.0));
   // Limit Switch
   limitSwitch = climberLeader.getForwardLimitSwitch();
   // Ratchet
   climberRatchet = new Relay(ClimberConstants.CLIMBER_RATCHET_ID, Relay.Direction.kForward);
   climberRatchet.set(Relay.Value.kOff);
 }


 @Override
 public void updateInputs(ClimberIOInputs inputs) {
   inputs.appliedVoltsLeader = climberLeader.getBusVoltage() * climberLeader.getAppliedOutput();
   inputs.currentAmpsLeader = climberLeader.getOutputCurrent();
   inputs.motorPercentOutput = (climberLeader.getAppliedOutput() * climberLeader.getBusVoltage()) / 12;
   inputs.ClimberLeaderPosition = climberLeaderEncoder.getPosition();
   inputs.appliedVoltsFollower = climberFollower.getBusVoltage() * climberFollower.getAppliedOutput();
   inputs.currentAmpsFollower = climberFollower.getOutputCurrent();
   inputs.ratchetLocked = climberRatchet.get().equals(Relay.Value.kOff);
   inputs.limitSwitchPressed = limitSwitch.isPressed();
 }


 @Override
 public void setMotorsOutputPercent(double percentOutput) {
   setLeaderMotorPercentOutput(percentOutput);
   setFollowerMotorPercentOutput(percentOutput);
 }


 public void setLeaderMotorPercentOutput(double percentOutput) {
   climberLeader.set(percentOutput);
 }


 public void setFollowerMotorPercentOutput(double percentOutput) {
   climberFollower.set(percentOutput);
 }


 public void setRatchetLocked(boolean locked) {
   climberRatchet.set(locked ? Relay.Value.kOff : Relay.Value.kOn);
 }


 @Override
 public boolean getLimitSwitch() {
   return limitSwitch.isPressed();
 }


 @Override
 public void resetPosition() {
   climberLeaderEncoder.setPosition(0);
 }
}
