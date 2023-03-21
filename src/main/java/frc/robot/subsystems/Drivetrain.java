// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.PhysicalConstants;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  private final WPI_TalonSRX frontLeftDrive = new WPI_TalonSRX(DrivetrainConstants.kFrontLeftMotorID);
  private final WPI_TalonSRX frontRightDrive = new WPI_TalonSRX(DrivetrainConstants.kFrontRightMotorID);

  private final WPI_VictorSPX backLeftDrive = new WPI_VictorSPX(DrivetrainConstants.kBackLeftMotorID);
  private final WPI_VictorSPX backRightDrive = new WPI_VictorSPX(DrivetrainConstants.kBackRightMotorID);

  private final MecanumDrive mecanumDrive = new MecanumDrive(frontLeftDrive, backLeftDrive, frontRightDrive, backLeftDrive);

  private final Encoder frontLeftEncoder = new Encoder(DrivetrainConstants.kFrontLeftEncoderA, DrivetrainConstants.kFrontLeftEncoderB, false, EncodingType.k4X);
  private final Encoder backLeftEncoder = new Encoder(DrivetrainConstants.kBackLeftEncoderA, DrivetrainConstants.kBackLeftEncoderB, false, EncodingType.k4X);
  private final Encoder frontRightEncoder = new Encoder(DrivetrainConstants.kFrontRightEncoderA, DrivetrainConstants.kFrontRightEncoderB, false, EncodingType.k4X);
  private final Encoder backRightEncoder = new Encoder(DrivetrainConstants.kBackRightEncoderA, DrivetrainConstants.kBackRightEncoderB, false, EncodingType.k4X);   

  private final AHRS gyro = new AHRS();
  
  private final MecanumDriveOdometry odometry =
      new MecanumDriveOdometry(PhysicalConstants.kDriveKinematics,
      gyro.getRotation2d(),
      new MecanumDriveWheelPositions());  


  public Drivetrain() {
        //gyro  
        new Thread(() -> {
          try {
              Thread.sleep(1000);
              gyro.reset();
          } catch (Exception e) {
          }
      }).start();
      
    //left side
    frontLeftDrive.configFactoryDefault();
    backLeftDrive.configFactoryDefault();

    frontLeftDrive.setNeutralMode(NeutralMode.Brake);
    backLeftDrive.setNeutralMode(NeutralMode.Brake);

    frontLeftDrive.setInverted(DrivetrainConstants.kFrontLeftMotorInverted);
    backLeftDrive.setInverted(DrivetrainConstants.kBackLeftMotorInverted);
 
    //right side
    frontRightDrive.configFactoryDefault();
    backRightDrive.configFactoryDefault();

    frontRightDrive.setNeutralMode(NeutralMode.Brake);
    backRightDrive.setNeutralMode(NeutralMode.Brake);

    frontRightDrive.setInverted(DrivetrainConstants.kFrontRightMotorInverted);
    backRightDrive.setInverted(DrivetrainConstants.kBackRightMotorInverted);

    //encoders

    frontLeftEncoder.setDistancePerPulse(PhysicalConstants.kDistancePerPulse);
    frontRightEncoder.setDistancePerPulse(PhysicalConstants.kDistancePerPulse);
    backLeftEncoder.setDistancePerPulse(PhysicalConstants.kDistancePerPulse);
    backRightEncoder.setDistancePerPulse(PhysicalConstants.kDistancePerPulse);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    odometry.update(gyro.getRotation2d(), getCurrentWheelDistances());

    SmartDashboard.putNumber("Robot Heading", getHeading());
    SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(gyro.getRotation2d(), getCurrentWheelDistances(), pose);
  }

   /**
   * Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1] and the linear
   * speeds have no effect on the angular speed.
   *
   * @param xSpeed Speed of the robot in the x direction (forward/backwards).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    if (fieldRelative) {
      mecanumDrive.driveCartesian(xSpeed, ySpeed, rot, gyro.getRotation2d());
    } else {
      mecanumDrive.driveCartesian(xSpeed, ySpeed, rot);
    }
  }

  public void autoDrive(double frontLeftVolts, double frontRightVolts, double backLeftVolts, double backRightVolts) {
    frontLeftDrive.setVoltage(frontLeftVolts);
    backLeftDrive.setVoltage(backLeftVolts);
    frontRightDrive.setVoltage(frontRightVolts);
    backRightDrive.setVoltage(backRightVolts);
  }


    /** Sets the drive MotorController to a voltage. */
    public void setDriveMotorControllersVolts(MecanumDriveMotorVoltages volts) {
      frontLeftDrive.setVoltage(volts.frontLeftVoltage);
      backLeftDrive.setVoltage(volts.rearLeftVoltage);
      frontRightDrive.setVoltage(volts.frontRightVoltage);
      backRightDrive.setVoltage(volts.rearRightVoltage);
    }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    frontLeftEncoder.reset();
    frontRightEncoder.reset();
    backLeftEncoder.reset();
    backRightEncoder.reset();
  }

  public Encoder getFrontLeftEncoder(){
    return frontLeftEncoder;
  }

  public Encoder getBackLeftEncoder(){
    return backLeftEncoder;
  }

  public Encoder getFrontRightEncoder(){
    return frontRightEncoder;
  }

  public Encoder getBackRightEncoder(){
    return backRightEncoder;
  }

  public MecanumDriveWheelSpeeds getCurrentWheelSpeeds() {
    return new MecanumDriveWheelSpeeds(
        frontLeftEncoder.getRate(),
        backLeftEncoder.getRate(),
        frontRightEncoder.getRate(),
        backRightEncoder.getRate());
  }

  public MecanumDriveWheelPositions getCurrentWheelDistances() {
    return new MecanumDriveWheelPositions(
        frontLeftEncoder.getDistance(),
        backLeftEncoder.getDistance(),
        frontRightEncoder.getDistance(),
        backRightEncoder.getDistance());
  }

  public void setMaxOutput(double maxOutput) {
    mecanumDrive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return gyro.getRotation2d().getDegrees();
  }

  public Rotation2d getRotation2d(){
    return Rotation2d.fromDegrees(getHeading());
  }



  public double getTurnRate() {
    return -gyro.getRate();
  }

  public boolean getFieldRelative() {
    return gyro.isConnected();
  }
}
