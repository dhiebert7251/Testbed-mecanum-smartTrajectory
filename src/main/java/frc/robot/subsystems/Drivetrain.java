// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

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
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  private ShuffleboardTab encoderTab, gyroTab, driveTab, driverCameraTab;

  private NetworkTableInstance ntInstance;


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
    
    /*
     * disable fieldRelative for testing purposes
     * 
  
    if (fieldRelative) {
      mecanumDrive.driveCartesian(xSpeed, ySpeed, rot, gyro.getRotation2d());
    } else {
      mecanumDrive.driveCartesian(xSpeed, ySpeed, rot);
    }


     */

    mecanumDrive.driveCartesian(xSpeed, ySpeed, rot);

    SmartDashboard.putNumber("Drive X Speed", xSpeed);
    SmartDashboard.putNumber("Drive Y Speed", ySpeed);
    SmartDashboard.putNumber("Drive Rotation", rot);
    SmartDashboard.putBoolean("Drive Field Relative", fieldRelative);
  }

  public void autoDrive(double frontLeftVolts, double frontRightVolts, double backLeftVolts, double backRightVolts) {
    frontLeftDrive.setVoltage(frontLeftVolts);
    backLeftDrive.setVoltage(backLeftVolts);
    frontRightDrive.setVoltage(frontRightVolts);
    backRightDrive.setVoltage(backRightVolts);

    SmartDashboard.putNumber("AutoDrive FL Volts", frontLeftVolts);
    SmartDashboard.putNumber("AutoDrive BL Volts", backLeftVolts);
    SmartDashboard.putNumber("Autodrive FR Volts", frontRightVolts);
    SmartDashboard.putNumber("AutoDrive BR Volts", backRightVolts);
  }


    /** Sets the drive MotorController to a voltage. */
    public void setDriveMotorControllersVolts(MecanumDriveMotorVoltages volts) {
      frontLeftDrive.setVoltage(volts.frontLeftVoltage);
      backLeftDrive.setVoltage(volts.rearLeftVoltage);
      frontRightDrive.setVoltage(volts.frontRightVoltage);
      backRightDrive.setVoltage(volts.rearRightVoltage);

      SmartDashboard.putNumber("setDriveMotor FL Volts", volts.frontLeftVoltage);
      SmartDashboard.putNumber("setDriveMotor BL Volts", volts.rearLeftVoltage);
      SmartDashboard.putNumber("setDriveMotor FR Volts", volts.frontRightVoltage);
      SmartDashboard.putNumber("setDriveMotor BR Volts", volts.rearRightVoltage);
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
  
    //ntInstance = NetworkTableInstance.getDefault();

    // Update telemetry 
    Shuffleboard.update();

    encoderTab = Shuffleboard.getTab("EncoderTab");

    ShuffleboardLayout frontLeftEncoderLayout = encoderTab //if this works switch the rest
      .getLayout("Front Left Encoder", BuiltInLayouts.kList)
      .withPosition(0,0)
      .withSize(2,2)
      .withProperties(Map.of("Label Position", "TOP"));

      frontLeftEncoderLayout
        .add("Raw", frontLeftEncoder.getRaw())
        .withWidget(BuiltInWidgets.kTextView);
      frontLeftEncoderLayout
        .add("Distance Per Pulse", frontLeftEncoder.getDistancePerPulse())
        .withWidget(BuiltInWidgets.kTextView);
      frontLeftEncoderLayout
        .add("Distance", frontLeftEncoder.getDistance())
        .withWidget(BuiltInWidgets.kTextView);
      frontLeftEncoderLayout
        .add("Forward?", frontLeftEncoder.getDirection())
        .withWidget(BuiltInWidgets.kToggleSwitch);


    ShuffleboardLayout backLeftEncoderLayout = Shuffleboard.getTab("EncoderTab")
      .getLayout("back Left Encoder", BuiltInLayouts.kList)
      .withPosition(0,3)
      .withSize(2,2)
      .withProperties(Map.of("Label Position", "TOP"));

      backLeftEncoderLayout
        .add("Raw",backLeftEncoder.getRaw())
        .withWidget(BuiltInWidgets.kTextView);
      backLeftEncoderLayout
        .add("Distance Per Pulse", backLeftEncoder.getDistancePerPulse())
        .withWidget(BuiltInWidgets.kTextView);
      backLeftEncoderLayout
        .add("Distance", backLeftEncoder.getDistance())
        .withWidget(BuiltInWidgets.kTextView);
      backLeftEncoderLayout
        .add("Forward?", backLeftEncoder.getDirection())
        .withWidget(BuiltInWidgets.kToggleSwitch);


        ShuffleboardLayout frontRightEncoderLayout = Shuffleboard.getTab("EncoderTab")
        .getLayout("Front Right Encoder", BuiltInLayouts.kList)
        .withPosition(3,0)
        .withSize(2,2)
        .withProperties(Map.of("Label Position", "TOP"));
  
        frontRightEncoderLayout
          .add("Raw", frontRightEncoder.getRaw())
          .withWidget(BuiltInWidgets.kTextView);
        frontRightEncoderLayout
          .add("Distance Per Pulse", frontRightEncoder.getDistancePerPulse())
          .withWidget(BuiltInWidgets.kTextView);
        frontRightEncoderLayout
          .add("Distance", frontRightEncoder.getDistance())
          .withWidget(BuiltInWidgets.kTextView);
        frontRightEncoderLayout
          .add("Forward?", frontRightEncoder.getDirection())
          .withWidget(BuiltInWidgets.kToggleSwitch);
  
  
      ShuffleboardLayout backRightEncoderLayout = Shuffleboard.getTab("EncoderTab")
        .getLayout("back Right Encoder", BuiltInLayouts.kList)
        .withPosition(3,3)
        .withSize(2,2)
        .withProperties(Map.of("Label Position", "TOP"));
  
        backRightEncoderLayout
          .add("Raw",backRightEncoder.getRaw())
          .withWidget(BuiltInWidgets.kTextView);
        backRightEncoderLayout
          .add("Distance Per Pulse", backRightEncoder.getDistancePerPulse())
          .withWidget(BuiltInWidgets.kTextView);
        backRightEncoderLayout
          .add("Distance", backRightEncoder.getDistance())
          .withWidget(BuiltInWidgets.kTextView);
        backRightEncoderLayout
          .add("Forward?", backRightEncoder.getDirection())
          .withWidget(BuiltInWidgets.kToggleSwitch);
  
/* 
      gyroTab = Shuffleboard.getTab("Gyro");
      gyroTab.add("Yaw, Pitch, Roll", BuiltInLayouts.kList)
              .withPosition(0, 0)
              .withSize(3, 2)
              .withProperties(Map.of("Label position", "TOP"))
              .add("Yaw", gyro::getYaw)
              .withWidget(BuiltInWidgets.kTextView)
              .add("Pitch", gyro::getPitch)
              .withWidget(BuiltInWidgets.kTextView)
              .add("Roll", gyro::getRoll)
              .withWidget(BuiltInWidgets.kTextView);


    //Gyro tab entries
    ShuffleboardTab gyroTab = Shuffleboard.getTab("GyroTab");
    ShuffleboardLayout headingLayout = 
      gyroTab.getLayout("Gyro Heading", BuiltInLayouts.kGrid)
             .withPosition(0, 0);
    
        headingLayout
          .add("Angle", gyro.getAngle())
          .withWidget(BuiltInWidgets.kDial);
          //SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());

        headingLayout
          .add("Heading", getHeading())
          .withWidget(BuiltInWidgets.kGyro);
    
      gyroTab
        .add("Gyro Rotation Rate", gyro.getRate());
        //SmartDashboard.putNumber("Gyro Rotation Rate", gyro.getRate());
    
    ShuffleboardLayout displacementLayout =
      gyroTab.getLayout("Displacement", BuiltInLayouts.kGrid)
             .withPosition(2, 0);

        displacementLayout
          .add("Disp X", gyro.getDisplacementX());
          //SmartDashboard.putNumber("Gyro Displacement X", gyro.getDisplacementX());

        displacementLayout
          .add("Disp Y", gyro.getDisplacementY());
          //SmartDashboard.putNumber("Gyro Displacement Y", gyro.getDisplacementY());

        displacementLayout
          .add("Disp Z", gyro.getDisplacementZ());
          //SmartDashboard.putNumber("Gyro Displacement Z", gyro.getDisplacementZ());

    ShuffleboardLayout yprLayout =
      gyroTab.getLayout("Displacement", BuiltInLayouts.kGrid)
             .withPosition(4, 0);          

        yprLayout
          .add("Gyro Yaw", gyro.getYaw());
          //SmartDashboard.putNumber("Gyro Yaw", gyro.getYaw());

        yprLayout
          .add("Gyro Pitch", gyro.getPitch());
          //SmartDashboard.putNuyro Pitch", gyro.getPitch());

        yprLayout
          .add("Gyro Roll", gyro.getRoll());
          //SmartDashboard.putNumber("Gmber("Gyro Roll", gyro.getRoll());

    ShuffleboardLayout velocityLayout =
      gyroTab.getLayout("Velocity", BuiltInLayouts.kList)
             .withPosition(6, 0);

        velocityLayout
          .add("Vel X", gyro.getVelocityX());
          //SmartDashboard.putNumber("Gyro Velocity X", gyro.getVelocityX());

        velocityLayout
          .add("Vel Y", gyro.getVelocityY());
          //SmartDashboard.putNumber("Gyro Velocity Y", gyro.getVelocityY());

        velocityLayout
          .add("Vel Z", gyro.getVelocityZ());
          //SmartDashboard.putNumber("Gyro Velocity Z", gyro.getVelocityZ());

    //Drive Tab entries
    ShuffleboardTab driveTab = Shuffleboard.getTab("DriveTab");

    driveTab
      .add("Mecanum Drive",mecanumDrive).withPosition(8,0);

    ShuffleboardLayout controllersLayout = 
      driveTab.getLayout("Controllers",BuiltInLayouts.kGrid).withPosition(0,0).withSize(2,2);
    
        controllersLayout
          //driveTab
          .add("FL", frontLeftDrive)
          .withWidget(BuiltInWidgets.kMotorController)
          .withPosition(0,0);
        controllersLayout
          //driveTab
          .add("BL", backLeftDrive)
          .withWidget(BuiltInWidgets.kMotorController)
          .withPosition(0,1);
        controllersLayout
          //driveTab
          .add("FR", frontRightDrive)
          .withWidget(BuiltInWidgets.kMotorController)
      .   withPosition(1,0);
        controllersLayout
          //driveTab
          .add("BR", backRightDrive)
          .withWidget(BuiltInWidgets.kMotorController)
          .withPosition(1,1);

    

    
    //Field Tab entries
    ShuffleboardTab fieldTab = Shuffleboard.getTab("FieldTab");
    fieldTab
      .add("Robot Location", getPose().getTranslation().toString());
      //SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());

    fieldTab
      .add("Robot Location (trimmed)", getPose().getTranslation().toString().replaceFirst(".*[,]",""));
      //SmartDashboard.putString("Robot Location (trimmed)", getPose().getTranslation().toString().replaceFirst(".*[,]",""));

*/
  }
  
}
