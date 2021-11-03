// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.logging.LogManager;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.constants.BlankConstants;
import frc.robot.constants.Robot2019Constants;

/**
 * Do NOT add any static variables to this class, or any initialization at all. Unless you know what
 * you are doing, do not modify this file except to change the parameter class to the startRobot
 * call.
 */
public final class Main {

  private Main() {}

  /**
   * Main initialization function. Do not perform any initialization here.
   *
   * <p>If you change your main robot class, change the parameter type.
   * @throws IOException
   */
  public static void main(String... args) throws IOException {
    RobotBase.startRobot(Robot::new);
  }
}
