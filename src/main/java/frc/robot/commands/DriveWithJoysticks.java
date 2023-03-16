// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Drivetrain;

public class DriveWithJoysticks extends CommandBase {
  private final Drivetrain m_drive;
  private final DoubleSupplier m_forward;
  private final DoubleSupplier m_strafe;
  private final DoubleSupplier m_rotation;

  public DriveWithJoysticks(Drivetrain dt, DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier rotation) {
    m_drive = dt;
    m_forward = forward;
    m_strafe = strafe;
    m_rotation = rotation;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = MathUtil.applyDeadband(m_forward.getAsDouble(), OperatorConstants.kControllerDeadband);
    double ySpeed = MathUtil.applyDeadband(m_strafe.getAsDouble(), OperatorConstants.kControllerDeadband);
    double rot = MathUtil.applyDeadband(m_rotation.getAsDouble(), OperatorConstants.kControllerDeadband);

    //shape the input
    xSpeed = xSpeed*xSpeed;
    ySpeed = ySpeed*ySpeed;
    rot = rot*rot;

    //apply speed limitter
    xSpeed = xSpeed * DrivetrainConstants.kDriveSpeedScale;
    ySpeed = ySpeed * DrivetrainConstants.kDriveSpeedScale;
    rot = rot * DrivetrainConstants.kDriveSpeedScale;

    m_drive.drive(xSpeed, ySpeed, rot, m_drive.getFieldRelative());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
