package frc.robot;

import frc.robot.constants.BlankConstants;
import frc.robot.constants.BriefcaseConstants;
import frc.robot.constants.Constants;
import frc.robot.constants.KitBot2022Constants;
import frc.robot.constants.Robot2019Constants;
import frc.robot.constants.Robot2022Constants;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;

public class RobotConstants {
  /*
    ------- Adding Robot File to New Roborio -------
    ssh into the Roborio
      This can be accomplished by opening git bash and running `$ sshadmin@roborio-467-frc.lan`
      or `$ ssh admin@10.4.67.2`
    Create a text file with the rio name
      This can be accomplished by running `echo "{Robot Name}" > /home/lvuser/robot`
      The robot name should be surrounded by quotes and should be lower case
    Then add the robot name to the code below and set the correct constants file
  */

  private static Constants constants = null;

  /** RobotConstants is a utility class, and should not be instantiated */
  private RobotConstants() {
    throw new IllegalStateException("Utility class");
  }

  /**
   * Initializes the constants file that is used
   *
   * @throws IOException reading of the /robot file fails
   */
  private static void initializeConstants() throws IOException {
    // search for robot file
    File file = new File(System.getProperty("user.home") + "/robot");
    if (!file.exists()) {
      // no robot file, add BlankConstants
      System.err.println("No roborio name file found");
      RobotConstants.set(new BlankConstants());
      return;
    }
    FileReader reader = new FileReader(file);
    BufferedReader br = new BufferedReader(reader);
    String name = br.readLine().toLowerCase();
    System.out.println("Name: " + name);
    switch (name) {
      case "turing":
        RobotConstants.set(new BriefcaseConstants());
        break;

      case "lovelace":
        RobotConstants.set(new Robot2019Constants());
        break;

      case "von neumann":
        RobotConstants.set(new Robot2022Constants());
        break;

      case "hopper":
        RobotConstants.set(new KitBot2022Constants());
        break;

      default:
        System.err.println("No valid roborio name found");
        RobotConstants.set(new BlankConstants());
        break;
    }
    System.out.println("Using constant file: " + RobotConstants.get().name());

    br.close();
  }

  /**
   * Gets the current constants file being used.
   *
   * @return a constants file
   */
  public static Constants get() {
    if (constants == null) {
      try {
        initializeConstants();
      } catch (IOException e) {
        e.printStackTrace();
      }
    }

    return constants;
  }

  /**
   * Sets the current constants file being used
   *
   * @param robot the constants file being used
   */
  public static void set(Constants robot) {
    constants = robot;
  }
}
