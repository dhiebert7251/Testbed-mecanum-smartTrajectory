// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  private final TalonSRX frontLeftDrive, frontRightDrive;
  private final VictorSPX backLeftDrive, backRightDrive;

  private final Encoder frontLeftEncoder, frontRightEncoder, backLeftEncoder, backRightEncoder;



  public Drivetrain() {
    //left side
    frontLeftDrive = new TalonSRX(DrivetrainConstants.kFrontLeftMotorID);
    backLeftDrive = new VictorSPX(DrivetrainConstants.kBackLeftMotorID);

    frontLeftEncoder = new Encoder(DrivetrainConstants.kFrontLeftEncoderA, DrivetrainConstants.kFrontLeftEncoderB);
    backLeftEncoder = new Encoder(DrivetrainConstants.kBackLeftEncoderA, DrivetrainConstants.kBackLeftEncoderB);    
   
    frontLeftDrive.configFactoryDefault();
    backLeftDrive.configFactoryDefault();

    frontLeftDrive.setNeutralMode(NeutralMode.Brake);
    backLeftDrive.setNeutralMode(NeutralMode.Brake);

    frontLeftDrive.setInverted(DrivetrainConstants.kFrontLeftMotorInverted);
    backLeftDrive.setInverted(DrivetrainConstants.kBackLeftMotorInverted);

    backLeftDrive.follow(frontLeftDrive);
 

    //right side
    frontRightDrive = new TalonSRX(DrivetrainConstants.kFrontRightMotorID);
    backRightDrive = new VictorSPX(DrivetrainConstants.kBackRightMotorID);

    frontRightEncoder = new Encoder(DrivetrainConstants.kFrontRightEncoderA, DrivetrainConstants.kFrontRightEncoderB);
    backRightEncoder = new Encoder(DrivetrainConstants.kBackRightEncoderA, DrivetrainConstants.kBackRightEncoderB); 

    frontRightDrive.configFactoryDefault();
    backRightDrive.configFactoryDefault();

    frontRightDrive.setNeutralMode(NeutralMode.Brake);
    backRightDrive.setNeutralMode(NeutralMode.Brake);

    frontRightDrive.setInverted(DrivetrainConstants.kFrontRightMotorInverted);
    backRightDrive.setInverted(DrivetrainConstants.kBackRightMotorInverted);

    backRightDrive.follow(frontRightDrive);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
