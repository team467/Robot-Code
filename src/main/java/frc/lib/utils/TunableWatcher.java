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
        DriverStationErrors.reportWarning("Adding tunable", false);
        tunables.add(
            new Tunable(parent, fieldName,
                new LoggedNetworkNumber("/Tunables/" + displayName, value.in(unit.get())),
                type, unit,
                setter, defaultValue)
        );
      } else {
        DriverStationErrors.reportWarning("Adding tunable non-Unit", false);
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
          double value = tunable.source.get();
          if (Measure.class.isAssignableFrom(tunable.type)) {
            if (!tunable.value.equals(tunable.unit.get().of(value))) {
              tunable.setter.invoke(tunable.unit.get().of(value));
              tunable.valueChanged = true;
              changedClasses.add(tunable.parent);
            } else {
              tunable.valueChanged = false;
            }
          } else {
            tunable.setter.invoke(value);
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
    public LoggedNetworkNumber source;
    public Class<?> type;
    public Optional<Unit> unit;
    public MethodHandle setter;
    public Object value;
    public boolean valueChanged = false;

    public Tunable(Class<?> parent, String name, LoggedNetworkNumber source, Class<?> type,
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
