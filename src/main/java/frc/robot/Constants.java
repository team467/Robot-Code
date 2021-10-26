// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.drive.MotorType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static boolean DRIVE_DUAL_MOTORS = true;
    public static int DRIVE_MOTOR_LEFT_LEADER_ID = 1;
    public static MotorType DRIVE_MOTOR_LEFT_LEADER_TYPE = MotorType.TALON_SRX;
    public static boolean DRIVE_MOTOR_LEFT_LEADER_INVERTED = false;

    public static int DRIVE_MOTOR_LEFT_FOLLOWER_ID = 2;
    public static MotorType DRIVE_MOTOR_LEFT_FOLLOWER_TYPE = MotorType.TALON_SRX;
    public static boolean DRIVE_MOTOR_LEFT_FOLLOWER_INVERTED = false;

    public static int DRIVE_MOTOR_RIGHT_LEADER_ID = 3;
    public static MotorType DRIVE_MOTOR_RIGHT_LEADER_TYPE = MotorType.TALON_SRX;
    public static boolean DRIVE_MOTOR_RIGHT_LEADER_INVERTED = false;

    public static int DRIVE_MOTOR_RIGHT_FOLLOWER_ID = 4;
    public static MotorType DRIVE_MOTOR_RIGHT_FOLLOWER_TYPE = MotorType.TALON_SRX;
    public static boolean DRIVE_MOTOR_RIGHT_FOLLOWER_INVERTED = false;

    public static boolean HAS_CLIMBER = false;
    public static int CLIMBER_MOTOR_ID = 2;
    public static double CLIMBER_SPEED = 0.1;
}
