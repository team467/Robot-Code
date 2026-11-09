package frc.lib.utils;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.MethodHandles;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkInput;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.units.Measure;
import org.wpilib.units.Unit;

public class TunableWatcher implements Runnable {

  private static final List<Tunable> tunables = new ArrayList<>();
  private static final Map<Class<?>, Runnable> tunableWatchers = new HashMap<>();

  public static void registerTunableWatcher(Class<?> clazz, Runnable consumer) {
    tunableWatchers.put(clazz, consumer);
  }

  public static void addTunable(Class<?> parent, String fieldName, String displayName,
      Class<?> type,
      Optional<Unit> unit,
      Object defaultValue) {
    DriverStationErrors.reportWarning("Add tunable called", false);
    if (unit.isEmpty() && defaultValue instanceof Measure) {
      throw new RuntimeException("Units must be provided when using a Measure tunable");
    }

    try {
      Field field = parent.getDeclaredField(fieldName);
      field.setAccessible(true);

      MethodHandles.Lookup lookup = MethodHandles.lookup();
      MethodHandle setter = lookup.unreflectSetter(field);

      if (defaultValue instanceof Measure value) {
        tunables.add(
            new Tunable(parent, fieldName,
                new LoggedNetworkNumber("/Tunables/" + displayName, value.in(unit.get())),
                type, unit,
                setter, defaultValue)
        );
      } else if (defaultValue instanceof Integer) {
        tunables.add(
            new Tunable(parent, fieldName,
                new LoggedNetworkNumber("/Tunables/" + displayName, (Integer) defaultValue),
                type, unit,
                setter, defaultValue));
      } else if (defaultValue instanceof Boolean) {
        tunables.add(
            new Tunable(parent, fieldName,
                new LoggedNetworkBoolean("/Tunables/" + displayName, (Boolean) defaultValue),
                type, unit,
                setter, defaultValue));
      } else {
        tunables.add(
            new Tunable(parent, fieldName,
                new LoggedNetworkNumber("/Tunables/" + displayName, (Double) defaultValue),
                type, unit,
                setter, defaultValue));
      }
    } catch (NoSuchFieldException | IllegalAccessException e) {
      throw new RuntimeException(e);
    }
  }

  @Override
  public void run() {
    while (true) {
      Set<Class<?>> changedClasses = new HashSet<>();
      for (var tunable : tunables) {
        try {
          if (tunable.source instanceof LoggedNetworkNumber source) {
            double value = source.get();
            if (tunable.setter.type().parameterType(0).equals(int.class)) {
              tunable.setter.invoke((int) value);
            } else if (Measure.class.isAssignableFrom(tunable.type)) {
              if (!tunable.value.equals(tunable.unit.get().of(value))) {
                tunable.setter.invoke(tunable.unit.get().of(value));
                changedClasses.add(tunable.parent);
              }
            } else {
              tunable.setter.invoke(value);
            }
          } else if (tunable.source instanceof LoggedNetworkBoolean source) {
            boolean value = source.get();
            if (!tunable.value.equals(value)) {
              tunable.setter.invoke(value);
              changedClasses.add(tunable.parent);
            }
          }
        } catch (Throwable e) {
          throw new RuntimeException(e);
        }
      }

      for (var clazz : changedClasses) {
        if (tunableWatchers.containsKey(clazz)) {
          tunableWatchers.get(clazz).run();
        }
      }

      Thread.yield();
    }
  }

  static class Tunable {

    public Class<?> parent;
    public String name;
    public LoggedNetworkInput source;
    public Class<?> type;
    public Optional<Unit> unit;
    public MethodHandle setter;
    public Object value;

    public Tunable(Class<?> parent, String name, LoggedNetworkInput source, Class<?> type,
        Optional<Unit> unit,
        MethodHandle setter, Object value) {
      this.parent = parent;
      this.name = name;
      this.source = source;
      this.type = type;
      this.unit = unit;
      this.setter = setter;
      this.value = value;
    }
  }
}
