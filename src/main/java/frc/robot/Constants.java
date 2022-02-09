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
    public static final int kRightMotor1ID = 1;
    public static final int kRightMotor2ID = 2;
    public static final int kLeftMotor1ID = 3;
    public static final int kLeftMotor2ID = 4;
    public static final int kGyroID = 5;

    public static final int[] kLeftEncoderPorts = new int[] { 0, 1 };
    public static final int[] kRightEncoderPorts = new int[] { 2, 3 };
    public static final boolean kLeftEncoderReversed = true;
    public static final boolean kRightEncoderReversed = false;

    // Intializes an object that can use the trackwidth (horizontal distance between
    // wheels) to convert from chassis speeds to wheel speeds
    public static final double kTrackWidthMeters = 0.62;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
        kTrackWidthMeters);

    // ? The counts per rotation of the encoders.
    public static final int kEncoderCPR = 2048;
    public static final double kWheelDiameterMeters = 0.1016;
    public static final double kGearReduction = 7.09;
    // The amount of meters traveled per pulse of the encoder
    public static final double kEncoderDistancePerPulse = ((kWheelDiameterMeters * Math.PI) / kGearReduction)
        / (double) kEncoderCPR;

    // ? Constants found by running a characterization routine (SysId).
    // These constants define how the robot reacts to input in the real world.
    // For example, how the robot actually accelerates on carpet.
    public static final double ksVolts = 0.55;
    public static final double kvVoltSecondsPerMeter = 2.43;
    public static final double kaVoltSecondsSquaredPerMeter = 0.15;

    // Kp constant found through Feedback Analysis in SysId.
    public static final double kPDriveVel = 2.53;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class AutoConstants {
    // ? Maximum values for path following.
    // The maximum speed must be less than the nominal free-speed of the robot.
    // The maximum acceleration is not crucial, but should stil be reasonable.
    // Nominal theoretical free speed = 4.79 m/s
    public static final double kMaxSpeedMetersPerSecond = 3.5;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;

    // ? Parameters for the RAMSETE controller used for path following.
    // These are passed directly to the controller on initialization.
    // These values are robot agnostic, meaning they should not change between
    // robots.
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  }
}
