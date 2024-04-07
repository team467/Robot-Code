package frc.lib.utils;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringPublisher;
import java.util.Arrays;
import java.util.Objects;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardInput;
import org.littletonrobotics.junction.networktables.LoggedDashboardString;

/** A string chooser for the dashboard where the options can be changed on-the-fly. */
public class SwitchableChooser implements LoggedDashboardInput {
  private static final String placeholder = "";

  private String[] options = new String[] {placeholder};
  private String active = placeholder;

  private final StringPublisher namePublisher;
  private final StringPublisher typePublisher;
  private final StringArrayPublisher optionsPublisher;
  private final StringPublisher defaultPublisher;
  private final StringPublisher activePublisher;
  private final StringPublisher selectedPublisher;
  private final LoggedDashboardString selectedInput;
  private String defaultOption;

  public SwitchableChooser(String name) {
    var table = NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable(name);
    namePublisher = table.getStringTopic(".name").publish();
    typePublisher = table.getStringTopic(".type").publish();
    optionsPublisher = table.getStringArrayTopic("options").publish();
    defaultPublisher = table.getStringTopic("default").publish();
    activePublisher = table.getStringTopic("active").publish();
    selectedPublisher = table.getStringTopic("selected").publish();
    selectedInput = new LoggedDashboardString(name + "/selected");
    Logger.registerDashboardInput(this);

    namePublisher.set(name);
    typePublisher.set("String Chooser");
    optionsPublisher.set(this.options);
    defaultPublisher.set(this.options[0]);
    activePublisher.set(this.options[0]);
    selectedPublisher.set(this.options[0]);
  }

  public SwitchableChooser(String name, String defaultOption) {
    this.defaultOption = defaultOption;
    var table = NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable(name);
    namePublisher = table.getStringTopic(".name").publish();
    typePublisher = table.getStringTopic(".type").publish();
    optionsPublisher = table.getStringArrayTopic("options").publish();
    defaultPublisher = table.getStringTopic("default").publish();
    activePublisher = table.getStringTopic("active").publish();
    selectedPublisher = table.getStringTopic("selected").publish();
    selectedInput = new LoggedDashboardString(name + "/selected");
    Logger.registerDashboardInput(this);

    namePublisher.set(name);
    typePublisher.set("String Chooser");
    optionsPublisher.set(this.options);
    defaultPublisher.set(this.defaultOption);
    activePublisher.set(this.defaultOption);
    selectedPublisher.set(this.defaultOption);
  }

  /** Updates the set of available options. */
  public void setOptions(String[] options) {
    if (Arrays.equals(options, this.options)) {
      return;
    }
    this.options = options.length == 0 ? new String[] {placeholder} : options;
    optionsPublisher.set(this.options);
    periodic();
  }

  public void setOptions(String[] options, String defaultOption) {
    this.defaultOption = defaultOption;
    if (Arrays.equals(options, this.options)) {
      return;
    }
    this.options = options.length == 0 ? new String[] {placeholder} : options;
    optionsPublisher.set(this.options);
    periodic();
  }
  /** Returns the selected option. */
  public String get() {
    return Objects.equals(active, placeholder) ? null : active;
  }

  public void periodic() {
    String selected = selectedInput.get();
    active = null;
    for (String option : options) {
      if (!option.equals(placeholder) && option.equals(selected)) {
        active = option;
      }
    }
    if (active == null) {
      if (this.defaultOption != null) {
        active = this.defaultOption;
      } else {
        active = options[0];
      }
      selectedPublisher.set(active);
    }
    defaultPublisher.set(active);
    activePublisher.set(active);
  }
}
