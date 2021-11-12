package frc.robot;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;

import frc.robot.constants.BlankConstants;
import frc.robot.constants.BriefcaseConstants;
import frc.robot.constants.Constants;
import frc.robot.constants.Robot2019Constants;

public class RobotConstants {
    private static Constants constants = null;

    private static void initalizeConstants() throws IOException {
        File file = new File(System.getProperty("user.home") + "/robot");
        if (!file.exists()) {
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
        
          default:
            System.err.println("No valid roborio name found");
            RobotConstants.set(new BlankConstants());
            break;
        }
    
    
        br.close();
      }
    
    public static Constants get() {
        if (constants == null) {
            try {
                initalizeConstants();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
        
        return constants;
    }

    public static void set(Constants robot) {
        constants = robot;
    }
}
