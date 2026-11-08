package frc.lib.utils;

import java.lang.reflect.InvocationTargetException;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.wpilib.units.Measure;
import org.wpilib.units.Unit;

public class Tunable<T> {

  private final String key;
  private final LoggedNetworkNumber number;
  private T defaultValue;
  private Unit valueUnit;

  public Tunable(String name, T defaultValue) {
    if (defaultValue instanceof Measure<?> value) {
      this.key = "/Tuning" + name;
      number = new LoggedNetworkNumber(key, value.magnitude());
      this.defaultValue = defaultValue;
      this.valueUnit = value.unit();
    } else if (defaultValue instanceof Double value) {
      this.key = "/Tuning" + name;
      number = new LoggedNetworkNumber(key, value);
      this.defaultValue = defaultValue;
    } else {
      throw new RuntimeException("Cannot use a TunableNumber with a non-unit or double type");
    }
  }

  public Tunable(String name) {
    this.key = "/Tuning" + name;
    number = new LoggedNetworkNumber(key, 0);
  }

  public T get() {
    try {
      if (defaultValue instanceof Measure<?> value) {
        return (T) valueUnit.of(number.get());
      } else {
        return (T) defaultValue.getClass().getConstructor(double.class).newInstance(number.get());
      }
    } catch (InstantiationException | IllegalAccessException | InvocationTargetException |
             NoSuchMethodException e) {
      throw new RuntimeException(e);
    }
  }
}
