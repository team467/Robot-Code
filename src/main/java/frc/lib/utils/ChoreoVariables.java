package frc.lib.utils;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.NewtonMeters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Torque;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.HashMap;

/** A class to assist in retreiving values from Choreo configuration files. */
public class ChoreoVariables {
  private static HashMap<String, Double> variableCache = new HashMap<>();
  private static HashMap<String, Pose2d> poseCache = new HashMap<>();
  private static boolean INITIALIZED = false;

  private static double getVal(JsonObject obj) {
    return obj.get("val").getAsDouble();
  }

  private static double getVal(JsonElement obj) {
    return getVal(obj.getAsJsonObject());
  }

  private static void initialize() {
    if (INITIALIZED) return;
    File choreoFile = new File(Filesystem.getDeployDirectory(), "choreo/AutoPaths.chor");
    try {
      var reader = new BufferedReader(new FileReader(choreoFile));
      String str = reader.lines().reduce("", (a, b) -> a + b);
      reader.close();
      JsonObject wholeChor = JsonParser.parseString(str).getAsJsonObject();
      JsonObject variables = wholeChor.get("variables").getAsJsonObject();
      JsonObject expressions = variables.get("expressions").getAsJsonObject();
      for (var entry : expressions.entrySet()) {
        variableCache.put(entry.getKey(), getVal(entry.getValue().getAsJsonObject().get("var")));
      }
      JsonObject poses = variables.get("poses").getAsJsonObject();
      for (var entry : poses.entrySet()) {
        JsonObject val = entry.getValue().getAsJsonObject();
        poseCache.put(
            entry.getKey(),
            new Pose2d(
                getVal(val.get("x")),
                getVal(val.get("y")),
                Rotation2d.fromRadians(getVal(val.get("heading")))));
      }
    } catch (Exception e) {
      DriverStation.reportError("Choreo Variable util fail!", e.getStackTrace());
      return;
    }
    INITIALIZED = true;
  }

  /**
   * Get a pose from Choreo.
   *
   * @param key The variable name in Choreo
   * @return The {@link Pose2d} with the given key
   */
  public static Pose2d getPose(String key) {
    initialize();
    return poseCache.get(key);
  }

  /**
   * Get a value as a double-precision floating point number from Choreo.
   *
   * @param key The variable name in Choreo
   * @return A double in the SI unit for that variable
   */
  public static double get(String key) {
    initialize();
    return variableCache.get(key).doubleValue();
  }

  /**
   * Get a value in {@link #Meters} from Choreo. NOTE: this works regardless of whether the value
   * actually is set as a length in Choreo.
   *
   * @param key The variable name in Choreo
   * @return A {@link Distance} object representing the length
   */
  public static Distance getLength(String key) {
    return Meters.of(get(key));
  }

  /**
   * Get a value in {@link #MetersPerSecond} from Choreo. NOTE: this works regardless of whether the
   * value actually is set as a linear velocity in Choreo.
   *
   * @param key The variable name in Choreo
   * @return A {@link LinearVelocity} object representing the linear velocity
   */
  public static LinearVelocity getLinearVelocity(String key) {
    return MetersPerSecond.of(get(key));
  }

  /**
   * Get a value in {@link #MetersPerSecondPerSecond} from Choreo. NOTE: this works regardless of
   * whether the value actually is set as a linear acceleration in Choreo.
   *
   * @param key The variable name in Choreo
   * @return A {@link LinearAcceleration} object representing the linear acceleration
   */
  public static LinearAcceleration getLinearAcceleration(String key) {
    return MetersPerSecondPerSecond.of(get(key));
  }

  /**
   * Get a value in {@link #Radians} from Choreo. NOTE: this works regardless of whether the value
   * actually is set as an angle in Choreo.
   *
   * @param key The variable name in Choreo
   * @return A {@link Angle} object representing the angle
   */
  public static Angle getAngle(String key) {
    return Radians.of(get(key));
  }

  /**
   * Get a rotation from Choreo. NOTE: this works regardless of whether the value actually is set as
   * an angle in Choreo.
   *
   * @param key The variable name in Choreo
   * @return A {@link Rotation2d} object representing the rotation
   */
  public static Rotation2d getRotation2d(String key) {
    return Rotation2d.fromRadians(get(key));
  }

  /**
   * Get a value in {@link #RadiansPerSecond} from Choreo. NOTE: this works regardless of whether
   * the value actually is set as a rotational velocity in Choreo.
   *
   * @param key The variable name in Choreo
   * @return A {@link AngularVelocity} object representing the rotational velocity
   */
  public static AngularVelocity getAngularVelocity(String key) {
    return RadiansPerSecond.of(get(key));
  }

  /**
   * Get a value in {@link #RadiansPerSecondPerSecond} from Choreo. NOTE: this works regardless of
   * whether the value actually is set as an angular acceleration in Choreo.
   *
   * @param key The variable name in Choreo
   * @return A {@link AngularAcceleration} object representing the angular acceleration
   */
  public static AngularAcceleration getAngularAcceleration(String key) {
    return RadiansPerSecondPerSecond.of(get(key));
  }

  /**
   * Get a value in {@link #Seconds} from Choreo. NOTE: this works regardless of whether the value
   * actually is set as a time in Choreo.
   *
   * @param key The variable name in Choreo
   * @return A {@link Time} object representing the time
   */
  public static Time getTime(String key) {
    return Seconds.of(get(key));
  }

  /**
   * Get a value in {@link #Kilograms} from Choreo. NOTE: this works regardless of whether the value
   * actually is set as a mass in Choreo.
   *
   * @param key The variable name in Choreo
   * @return A {@link Mass} object representing the mass
   */
  public static Mass getMass(String key) {
    return Kilograms.of(get(key));
  }

  /**
   * Get a value in {@link #NewtonMeters} from Choreo. NOTE: this works regardless of whether the
   * value actually is set as a torque in Choreo.
   *
   * @param key The variable name in Choreo
   * @return A {@link Torque} object representing the torque
   */
  public static Torque getTorque(String key) {
    return NewtonMeters.of(get(key));
  }

  /**
   * Get a value in {@link #KilogramSquareMeters} from Choreo. NOTE: this works regardless of
   * whether the value actually is set as an MOI in Choreo.
   *
   * @param key The variable name in Choreo
   * @return A {@link MomentOfInertia} object representing the MOI
   */
  public static MomentOfInertia getMOI(String key) {
    return KilogramSquareMeters.of(get(key));
  }

  /**
   * Deinitialize this - it will reinitialize later if necessary. This method cleans up memory by
   * removing the variable and pose caches used to retrieve values - when something is read the
   * whole file is read into these hash maps and kept and reused if more variables are read. This
   * resets those hash maps.
   */
  public static void deinitialize() {
    variableCache = new HashMap<>();
    poseCache = new HashMap<>();
    INITIALIZED = false;
    System.gc();
  }
}
