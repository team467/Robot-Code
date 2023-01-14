package frc.robot;

import frc.robot.constants.CompBotConstants;
import frc.robot.constants.Constants;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;

public class RobotConstants {
  /** TODO: Manually change this for simulation and Replay. */
  private static Constants constants = new CompBotConstants();

  private RobotConstants() {
    throw new IllegalStateException("Utility class");
  }

  private static void initConstants() throws IOException {
    // Not yet implemented with 2023 robots, set manually above
    if (constants == null) {
      File file = new File(System.getProperty("user.home") + "/robot");
      if (!file.exists()) {
        throw new IOException("No roborio name file found, add it or change robot var manually.");
      }
      FileReader reader = new FileReader(file);
      try (BufferedReader br = new BufferedReader(reader)) {
        String name = br.readLine().toLowerCase();
        System.out.println("Name: " + name);
        switch (name) {
          case "comp":
            RobotConstants.set(new CompBotConstants());
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
        throw new RuntimeException(e); // No compilation warnings
      }
    }

    return constants;
  }

  public static void set(Constants robot) {
    constants = robot;
  }
}
