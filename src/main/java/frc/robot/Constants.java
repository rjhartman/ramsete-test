// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final int RIGHT_LEAD_MOTOR_ID = 1;
    public static final int RIGHT_FOLLOWER_MOTOR_ID = 2;
    public static final int LEFT_LEAD_MOTOR_ID = 3;
    public static final int LEFT_FOLLOWER_MOTOR_ID = 4;
    public static final int GYRO_ID = 5;

    public static final boolean LEFT_SIDE_REVERSED = true;
    public static final boolean RIGHT_SIDE_REVERSED = false;

    // Intializes an object that can use the trackwidth (horizontal distance between
    // wheels) to convert from chassis speeds to wheel speeds
    public static final double TRACK_WIDTH = 0.62;
    public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(TRACK_WIDTH);

    // ? The counts per rotation of the drive encoders.
    public static final int ENCODER_CPR = 2048;
    public static final double WHEEL_DIAMETER = 0.1016;
    public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    public static final double GEAR_REDUCTION = 7.09;

    // The amount of meters traveled per pulse of the encoder
    public static final double ENCODER_DISTANCE_PER_PULSE = (WHEEL_CIRCUMFERENCE / GEAR_REDUCTION)
        / (double) ENCODER_CPR;

    // ? Constants found by running a characterization routine (SysId).
    // Feed forward analysis, defines how the robot responds to inputs to the motor
    // controllers.
    public static final double KS = 0.61;
    public static final double KV = 2.35;
    public static final double KA = 0.34;

    // Kp constant found through Feedback Analysis in SysId.
    public static final double KP = 2.46;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class AutoConstants {
    // ? Maximum values for path following.
    // The maximum speed must be less than the nominal free-speed of the robot.
    // The maximum acceleration is not crucial, but should stil be reasonable.
    // Nominal theoretical free speed = 4.79 m/s
    public static final double MAX_SPEED = 2;
    public static final double MAX_ACCELERATION = 2;

    // ? Parameters for the RAMSETE controller used for path following.
    // These are passed directly to the controller on initialization.
    // These values are robot agnostic, meaning they should not change between
    // robots.
    public static final double RAMSETE_B = 2;
    public static final double RAMSETE_ZETA = 0.7;
  }
}
