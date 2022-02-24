// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.motorcontrol.Luna_TalonFX;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  private final Luna_TalonFX leftLeadMotor = new Luna_TalonFX(DriveConstants.LEFT_LEAD_MOTOR_ID);
  private final Luna_TalonFX leftFollowerMotor = new Luna_TalonFX(DriveConstants.LEFT_FOLLOWER_MOTOR_ID);
  private final Luna_TalonFX rightLeadMotor = new Luna_TalonFX(DriveConstants.RIGHT_LEAD_MOTOR_ID);
  private final Luna_TalonFX rightFollowerMotor = new Luna_TalonFX(DriveConstants.RIGHT_FOLLOWER_MOTOR_ID);

  // The motors on the left side of the drive.
  private final MotorControllerGroup leftMotors = new MotorControllerGroup(
      leftLeadMotor,
      leftFollowerMotor);

  // The motors on the right side of the drive.
  private final MotorControllerGroup rightMotors = new MotorControllerGroup(
      rightLeadMotor,
      rightFollowerMotor);

  // The robot's drive
  private final DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);

  // The gyro sensor
  private final Gyro gyro = new WPI_PigeonIMU(0);

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry odometry;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    leftMotors.setInverted(DriveConstants.LEFT_SIDE_REVERSED);
    rightMotors.setInverted(DriveConstants.RIGHT_SIDE_REVERSED);

    // Reverse the encoders on the left/right motor,
    leftLeadMotor.setSensorPhase(DriveConstants.LEFT_SIDE_REVERSED);
    leftFollowerMotor.setSensorPhase(DriveConstants.LEFT_SIDE_REVERSED);
    rightLeadMotor.setSensorPhase(DriveConstants.RIGHT_SIDE_REVERSED);
    rightFollowerMotor.setSensorPhase(DriveConstants.RIGHT_SIDE_REVERSED);

    // Sets the distance per pulse for the encoders
    // m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    // m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);

    resetEncoders();
    odometry = new DifferentialDriveOdometry(gyro.getRotation2d());
  }

  @Override
  public void periodic() {

    if (IOConstants.ENABLE_DIAGNOSTICS) {
      SmartDashboard.putString("Encoder Distances (m)",
          "L: " + leftLeadMotor.getDistance() + " R: " + rightLeadMotor.getDistance());
      SmartDashboard.putString("Encoder Velocities (m/s)",
          "L: " + leftLeadMotor.getRate() + " R: " + rightLeadMotor.getRate());
      SmartDashboard.putString("Gyro Rotation (deg)", this.gyro.getRotation2d().getDegrees() + "");
      SmartDashboard.putString("Pose", this.getPose().toString());
    }

    // Update the odometry in the periodic block
    odometry.update(gyro.getRotation2d(),
        leftLeadMotor.getDistance(),
        rightLeadMotor.getDistance());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        leftLeadMotor.getRate(),
        rightLeadMotor.getRate());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, gyro.getRotation2d());
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    drive.arcadeDrive(fwd, rot);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMotors.setVoltage(leftVolts);
    rightMotors.setVoltage(rightVolts);
    drive.feed();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    leftLeadMotor.reset();
    leftFollowerMotor.reset();
    rightLeadMotor.reset();
    rightFollowerMotor.reset();
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    // Flip the encoder on the left side
    return (leftLeadMotor.getDistance() + leftFollowerMotor.getDistance() + rightLeadMotor.getDistance()
        + rightFollowerMotor.getDistance()) / 4.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  // public Encoder getLeftEncoder() {
  // return m_leftEncoder;
  // }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  // public Encoder getRightEncoder() {
  // return m_rightEncoder;
  // }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more
   * slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {

  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    // return m_gyro.getRotation2d().getDegrees();
    return gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -gyro.getRate();
  }
}
