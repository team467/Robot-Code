package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.BriefcaseConstants;
import frc.robot.constants.CompBotConstants;
import frc.robot.constants.Constants;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;

public class RobotConstants {
  private static Constants constants = new CompBotConstants();
  public static final boolean TUNING_MODE = false;

  private RobotConstants() {
    throw new IllegalStateException("Utility class");
  }

  private static void initConstants() throws IOException {
    if (constants == null) {
      File file = new File(System.getProperty("user.home") + "/robot");
      if (!file.exists()) {
        throw new IOException(
            "No roborio name file found, add it or change RobotConstants.constants var manually.");
      }
      FileReader reader = new FileReader(file);
      try (BufferedReader br = new BufferedReader(reader)) {
        String name = br.readLine().toLowerCase();
        System.out.println("Name: " + name);
        switch (name) {
          case "von neumann":
            RobotConstants.set(new CompBotConstants());
            break;
          case "turing":
            RobotConstants.set(new BriefcaseConstants());
            break;
          default:
            throw new IOException("Invalid roborio name found");
        }
        System.out.println("Using constant file: " + get().getClass().getName());
        reader.close();
      }
    }
  }

  public static Constants get() {
    if (constants == null) {
      try {
        initConstants();
      } catch (IOException e) {
        DriverStation.reportError(
            "[RobotConstants] Error initializing constants!", e.getStackTrace());
        throw new RuntimeException(e); // No compilation warnings
      }
    }

    return constants;
  }

  public static void set(Constants robot) {
    constants = robot;
  }
}
