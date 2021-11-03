// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.constants.BlankConstants;
import frc.robot.constants.Robot2019Constants;

/**
 * Do NOT add any static variables to this class, or any initialization at all. Unless you know what
 * you are doing, do not modify this file except to change the parameter class to the startRobot
 * call.
 */
public final class Main {
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
        RobotConstants.set(new Robot2019Constants());
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


  private Main() {}

  /**
   * Main initialization function. Do not perform any initialization here.
   *
   * <p>If you change your main robot class, change the parameter type.
   * @throws IOException
   */
  public static void main(String... args) throws IOException {
    initalizeConstants();
    RobotBase.startRobot(Robot::new);
  }
}
