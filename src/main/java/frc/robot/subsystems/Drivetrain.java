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
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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

  private final MecanumDrive mecanumDrive = new MecanumDrive(frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive);

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


    telemetry();
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

  public void telemetry(){
    //Shuffleboard.selectTab("DriveTab");
    //Shuffleboard.selectTab("GyroTab");
    //Shuffleboard.selectTab("PowerTab");public void telemetry(){
    
    // Update telemetry 
    Shuffleboard.update();

    //Encoder Tab entries
    Shuffleboard.getTab("EncoderTab")
      .getLayout("Front Left Encoder", BuiltInLayouts.kList)
      .add("Raw", frontLeftEncoder.getRaw());
      //SmartDashboard.putNumber("Left Encoder Raw", frontLeftEncoder.getRaw());
    Shuffleboard.getTab("EncoderTab")
      .getLayout("Front Left Encoder", BuiltInLayouts.kList)
      .add("Distance Per Pulse", frontLeftEncoder.getDistancePerPulse());
    Shuffleboard.getTab("EncoderTab")
      .getLayout("Front Left Encoder", BuiltInLayouts.kList)
      .add("Distance", frontLeftEncoder.getDistance());
      //SmartDashboard.putNumber("Left Encoder Distance", frontLeftEncoder.getDistance());
    Shuffleboard.getTab("EncoderTab")
      .getLayout("Front Left Encoder", BuiltInLayouts.kList)
      .add("Rate", frontLeftEncoder.getRate());
      //SmartDashboard.putNumber("Left Encoder Rate", frontLeftEncoder.getRate());
    Shuffleboard.getTab("EncoderTab")
      .getLayout("Front Left Encoder", BuiltInLayouts.kList)
      .add("Forward?", frontLeftEncoder.getDirection())
      .withWidget(BuiltInWidgets.kToggleSwitch);
      //SmartDashboard.putBoolean("Left Encoder Forward", frontLeftEncoder.getDirection()); //is this useful?

    Shuffleboard.getTab("EncoderTab")
      .getLayout("Front Right Encoder", BuiltInLayouts.kList)
      .add("Raw", frontRightEncoder.getRaw());
      //SmartDashboard.putNumber("Right Encoder Raw", frontRightEncoder.getRaw());
    Shuffleboard.getTab("EncoderTab")
      .getLayout("Front Right Encoder", BuiltInLayouts.kList)
      .add("Distance Per Pulse", frontRightEncoder.getDistancePerPulse());
    Shuffleboard.getTab("EncoderTab")
      .getLayout("Front Right Encoder", BuiltInLayouts.kList)
      .add("Distance", frontRightEncoder.getDistance());
      //SmartDashboard.putNumber("Right Encoder Distance", frontRightEncoder.getDistance());
    Shuffleboard.getTab("EncoderTab")
      .getLayout("Front Right Encoder", BuiltInLayouts.kList)
      .add("Rate", frontRightEncoder.getRate());
      //SmartDashboard.putNumber("Right Encoder Rate", frontRightEncoder.getRate());
    Shuffleboard.getTab("EncoderTab")
      .getLayout("Front Right Encoder", BuiltInLayouts.kList)
      .add("Forward?", frontRightEncoder.getDirection())
      .withWidget(BuiltInWidgets.kToggleSwitch);
      //SmartDashboard.putBoolean("Right Encoder Forward", frontRightEncoder.getDirection()); //is this useful?

    Shuffleboard.getTab("EncoderTab")
      .getLayout("Back Left Encoder", BuiltInLayouts.kList)
      .add("Raw", backLeftEncoder.getRaw());
      //SmartDashboard.putNumber("Back Left Encoder Raw", backLeftEncoder.getRaw());
    Shuffleboard.getTab("EncoderTab")
      .getLayout("Back Left Encoder", BuiltInLayouts.kList)
      .add("Distance Per Pulse", backLeftEncoder.getDistancePerPulse());
    Shuffleboard.getTab("EncoderTab")
      .getLayout("Back Left Encoder", BuiltInLayouts.kList)
      .add("Distance", backLeftEncoder.getDistance());
      //SmartDashboard.putNumber("Back Left Encoder Distance", backLeftEncoder.getDistance());
    Shuffleboard.getTab("EncoderTab")
      .getLayout("Back Left Encoder", BuiltInLayouts.kList)
      .add("Rate", backLeftEncoder.getRate());
      //SmartDashboard.putNumber("Back Left Encoder Rate", backLeftEncoder.getRate());
    Shuffleboard.getTab("EncoderTab")
      .getLayout("Back Left Encoder", BuiltInLayouts.kList)
      .add("Forward?", backLeftEncoder.getDirection())
      .withWidget(BuiltInWidgets.kToggleSwitch);
      //SmartDashboard.putBoolean("Back Left Encoder Forward", backLeftEncoder.getDirection()); //is this useful?

    Shuffleboard.getTab("EncoderTab")
      .getLayout("Back Right Encoder", BuiltInLayouts.kList)
      .add("Raw", backRightEncoder.getRaw());
      //SmartDashboard.putNumber("Back Right Encoder Raw", backRightEncoder.getRaw());
    Shuffleboard.getTab("EncoderTab")
      .getLayout("Back Right Encoder", BuiltInLayouts.kList)
      .add("Distance Per Pulse", backRightEncoder.getDistancePerPulse());
    Shuffleboard.getTab("EncoderTab")
      .getLayout("Back Right Encoder", BuiltInLayouts.kList)
      .add("Distance", backRightEncoder.getDistance());
      //SmartDashboard.putNumber("Back Right Encoder Distance", backRightEncoder.getDistance());
    Shuffleboard.getTab("EncoderTab")
      .getLayout("Back Right Encoder", BuiltInLayouts.kList)
      .add("Rate", backRightEncoder.getRate());
      //SmartDashboard.putNumber("Back Right Encoder Rate", backRightEncoder.getRate());
    Shuffleboard.getTab("EncoderTab")
      .getLayout("Back Right Encoder", BuiltInLayouts.kList)
      .add("Forward?", backRightEncoder.getDirection())
      .withWidget(BuiltInWidgets.kToggleSwitch);
      //SmartDashboard.putBoolean("Back Right Encoder Forward", backRightEncoder.getDirection()); //is this useful?


    //Gyro tab entries
    Shuffleboard.getTab("GyroTab")
      .getLayout("Heading", BuiltInLayouts.kGrid)
      .add("Angle", gyro.getAngle())
      .withWidget(BuiltInWidgets.kDial);
      //SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());\
    Shuffleboard.getTab("GyroTab")
      .getLayout("Heading", BuiltInLayouts.kGrid)
      .add("Heading", getHeading())
      .withWidget(BuiltInWidgets.kGyro);
    
    Shuffleboard.getTab("GyroTab")
      .add("Gyro Rotation Rate", gyro.getRate());
      //SmartDashboard.putNumber("Gyro Rotation Rate", gyro.getRate());

    Shuffleboard.getTab("GyroTab")
      .getLayout("Displacement", BuiltInLayouts.kGrid)
      .add("X", gyro.getDisplacementX());
      //SmartDashboard.putNumber("Gyro Displacement X", gyro.getDisplacementX());
    Shuffleboard.getTab("GyroTab")
      .getLayout("Displacement", BuiltInLayouts.kGrid)
      .add("Y", gyro.getDisplacementY());
      //SmartDashboard.putNumber("Gyro Displacement Y", gyro.getDisplacementY());
    Shuffleboard.getTab("GyroTab")
      .getLayout("Displacement", BuiltInLayouts.kGrid)
      .add("Z", gyro.getDisplacementZ());
      //SmartDashboard.putNumber("Gyro Displacement Z", gyro.getDisplacementZ());

    Shuffleboard.getTab("GyroTab")
      .getLayout("YPR", BuiltInLayouts.kGrid)
      .add("Gyro Yaw", gyro.getYaw());
      //SmartDashboard.putNumber("Gyro Yaw", gyro.getYaw());
    Shuffleboard.getTab("GyroTab")
      .getLayout("YPR", BuiltInLayouts.kGrid)
      .add("Gyro Pitch", gyro.getPitch());
      //SmartDashboard.putNuyro Pitch", gyro.getPitch());
    Shuffleboard.getTab("GyroTab")
      .getLayout("YPR", BuiltInLayouts.kGrid)
      .add("Gyro Roll", gyro.getRoll());
      //SmartDashboard.putNumber("Gmber("Gyro Roll", gyro.getRoll());


    Shuffleboard.getTab("GyroTab")
      .getLayout("Velocity")
      .add("X", gyro.getVelocityX());
      //SmartDashboard.putNumber("Gyro Velocity X", gyro.getVelocityX());
    Shuffleboard.getTab("GyroTab")
      .getLayout("Velocity")
      .add("Y", gyro.getVelocityY());
      //SmartDashboard.putNumber("Gyro Velocity Y", gyro.getVelocityY());
    Shuffleboard.getTab("GyroTab")
      .getLayout("Velocity")
      .add("Z", gyro.getVelocityZ());
      //SmartDashboard.putNumber("Gyro Velocity Z", gyro.getVelocityZ());

    //Drive Tab entries
    Shuffleboard.getTab("DriveTab")
      .getLayout("Controllers",BuiltInLayouts.kGrid)
      .add("FL", frontLeftDrive)
      .withWidget(BuiltInWidgets.kMotorController);
    Shuffleboard.getTab("DriveTab")
      .getLayout("Controllers",BuiltInLayouts.kGrid)
      .add("BL", backLeftDrive)
      .withWidget(BuiltInWidgets.kMotorController);
    Shuffleboard.getTab("DriveTab")
      .getLayout("Controllers",BuiltInLayouts.kGrid)
      .add("FR", frontRightDrive)
      .withWidget(BuiltInWidgets.kMotorController);
    Shuffleboard.getTab("DriveTab")
      .getLayout("Controllers",BuiltInLayouts.kGrid)
      .add("BR", backRightDrive)
      .withWidget(BuiltInWidgets.kMotorController);

    
    //Field Tab entries
 

    Shuffleboard.getTab("FieldTab")
      .add("Robot Location", getPose().getTranslation().toString());
      //SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());

    Shuffleboard.getTab("FieldTab")
      .add("Robot Location (trimmed)", getPose().getTranslation().toString().replaceFirst(".*[,]",""));
      //SmartDashboard.putString("Robot Location (trimmed)", getPose().getTranslation().toString().replaceFirst(".*[,]",""));

      
  }
  
}
