// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Constants;
import frc.robot.sensors.RomiGyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// TODO: The gyro provides 3 angles; the angle we're most likely looking for 
// TODO: is either Z or Y depending on how the gyro proccesses angles.
public class Drivetrain extends SubsystemBase {
  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Spark m_leftMotor = new Spark(0);
  private final Spark m_rightMotor = new Spark(1);

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);

  // Set up the RomiGyro
  private final RomiGyro m_gyro = new RomiGyro();

  // Set up the BuiltInAccelerometer
  private final BuiltInAccelerometer m_accelerometer = new BuiltInAccelerometer();

  // Set up the differential drive controller
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotor, m_rightMotor);

  // Set up the differential drive odometry
  private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(normalizeAngle(m_gyro.getAngleZ()));

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse(Constants.Drivetrain.kEncoderDistancePerPulse);
    m_rightEncoder.setDistancePerPulse(Constants.Drivetrain.kEncoderDistancePerPulse);
    resetEncoders();
  }

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    m_drive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotor.setVoltage(leftVolts);
    m_rightMotor.setVoltage(rightVolts);
    m_drive.feed();
  }

  /** Reset the encoders. */
  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public int getLeftEncoderCount() {
    return m_leftEncoder.get();
  }

  public int getRightEncoderCount() {
    return m_rightEncoder.get();
  }

  public double getLeftDistanceInches() {
    return m_leftEncoder.getDistance();
  }

  public double getRightDistanceInches() {
    return m_rightEncoder.getDistance();
  }

  public double getAverageDistanceInch() {
    return (getLeftDistanceInches() + getRightDistanceInches()) / 2.0;
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }

  /** The acceleration of the Romi along the X-axis in Gs. */
  public double getAccelX() {
    return m_accelerometer.getX();
  }

  /** The acceleration of the Romi along the Y-axis in Gs. */
  public double getAccelY() {
    return m_accelerometer.getY();
  }

  /** The acceleration of the Romi along the Z-axis in Gs */
  public double getAccelZ() {
    return m_accelerometer.getZ();
  }

  /** Reset the odometry to a new anchor point. */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, normalizeAngle(m_gyro.getAngleZ()));
  }

  /** The combined X-axis & Y-axis positions of the Romi in meters. */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /** The position of the romi along the X-asix in meters. */
  public double getPoseX() {
    return m_odometry.getPoseMeters().getX();
  }

  /** The position of the romi along the Y-asix in meters. */
  public double getPoseY() {
    return m_odometry.getPoseMeters().getY();
  }

  /** Reset the gyro. */
  public void resetGyro() {
    m_gyro.reset();
  }

  /** The acceleration of the Romi along the X-axis in Gs */
  public double getGyroAngleX() {
    return m_gyro.getAngleX();
  }

  /** The current angle of the Romi around the Y-axis in degrees. */
  public double getGyroAngleY() {
    return m_gyro.getAngleY();
  }

  /** The current angle of the Romi around the Z-axis in degrees. */
  public double getGyroAngleZ() {
    return m_gyro.getAngleZ();
  }

  /** Converts angles from degrees to an abstract rotation2d. */
  public Rotation2d normalizeAngle(double angle) {
    return Rotation2d.fromDegrees(angle);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(normalizeAngle(m_gyro.getAngleZ()), Units.inchesToMeters(getLeftDistanceInches()), Units.inchesToMeters(getRightDistanceInches()));
  }
}
