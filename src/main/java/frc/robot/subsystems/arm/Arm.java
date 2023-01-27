
package frc.robot.subsystems.arm;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class Arm extends SubsystemBase {

  private static final CANSparkMax armExtendMotor = new CANSparkMax (0, MotorType.kBrushless);
  private static final CANSparkMax armRotateMotor = new CANSparkMax(1,MotorType.kBrushless);
  public static final double EXTEND_TOLERANCE_INCHES = 2.0;
  public static final double ROTATE_TOLERANCE_DEGREES = 2.0;

  private double distanceTargetInches = 0;
  private double rotateTargetDegrees = 0;
  
  private boolean enabled = false;

  public Arm() {
    super();
    
  }

  public void enable() {
    enabled = true;
  }

  public void disable() {
    stop();
    enabled = false;
  }

  public boolean isEnabled() {
    return enabled;
  }

  
  public void stop() {
    armExtendMotor.set(0.0);
    armExtendMotor.set(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // TODO: Proper conversions
    armExtendMotor.getEncoder().setPosition(distanceTargetInches);
    armRotateMotor.getEncoder().setPosition(rotateTargetDegrees);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
  }

  public void extendAndRotate(double distanceTargetInches, double rotateTargetDegrees) {
    this.distanceTargetInches = distanceTargetInches;
    this.rotateTargetDegrees = rotateTargetDegrees;
  }

  public boolean isStopped() {
    if (armExtendMotor.getEncoder().getVelocity() <= 0.1
      && armRotateMotor.getEncoder().getVelocity() <= 0.1)
      return true;
    return false;
  }

  public boolean finished() {
    double currentDistance = armExtendMotor.getEncoder().getPosition(); // NEEDS CONVERSION
    // TODO: CHANGE to Lidar

    double currentAngle = armRotateMotor.getEncoder().getPosition(); // NEEDS CONVERSION

    if (currentDistance >= (distanceTargetInches - EXTEND_TOLERANCE_INCHES)
      && (currentDistance <= (distanceTargetInches + EXTEND_TOLERANCE_INCHES) 
      && (currentAngle >= (rotateTargetDegrees - EXTEND_TOLERANCE_INCHES)
      && (currentAngle <= (rotateTargetDegrees + EXTEND_TOLERANCE_INCHES)
      )))) {
        return true;
      }

    return false;
  }

  
}
