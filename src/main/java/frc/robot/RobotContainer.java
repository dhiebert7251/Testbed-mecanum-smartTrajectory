// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PhysicalConstants;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.subsystems.Drivetrain;

import java.util.List;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  Thread m_visionThread;


  // Subsystems are defined here...
  private final Drivetrain m_driveTrain = new Drivetrain();

  // Commands are defined here...


  // Controllers
  // Replace with CommandPS4Controller or CommandJoystick if needed

  private final Joystick m_driverController =
    new Joystick(OperatorConstants.kDriverControllerPort);


  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();






  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    //Camera
    //driverCam = CameraServer.startAutomaticCapture();

    
    m_driveTrain.setDefaultCommand(
        // Left stick robot position, field oriented
        // Right stick rotation
        new DriveWithJoysticks(
          m_driveTrain, 
          () -> m_driverController.getRawAxis(OperatorConstants.kDriverLeftY), //does this need to be negative?
          () -> m_driverController.getRawAxis(OperatorConstants.kDriverLeftX),
          () -> m_driverController.getRawAxis(OperatorConstants.kDriverRightX))
      );

    // Add commands to the autonomous command chooser
    m_chooser.setDefaultOption("Trajectory 1", trajectory1());
    //m_chooser.addOption();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new Trigger(m_exampleSubsystem::exampleCondition)
    //    .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_chooser.getSelected();
  }

  public Command trajectory1() {
    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                PhysicalConstants.kMaxSpeed,
                PhysicalConstants.kMaxAccel)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(PhysicalConstants.kDriveKinematics);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(Units.feetToMeters(10), 0, new Rotation2d(0)),
            config);

    MecanumControllerCommand mecanumControllerCommand =
        new MecanumControllerCommand(
            exampleTrajectory,
            m_driveTrain::getPose,
            AutoConstants.kFeedforward,
            PhysicalConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints),
                // Needed for normalizing wheel speeds
                PhysicalConstants.kMaxSpeed,
                // Velocity PID's
                new PIDController(AutoConstants.kPFrontLeftVel, AutoConstants.kIFrontLeftVel, AutoConstants.kDFrontLeftVel),
                new PIDController(AutoConstants.kPBackLeftVel, AutoConstants.kIBackLeftVel, AutoConstants.kDBackLeftVel),
                new PIDController(AutoConstants.kPFrontRightVel, AutoConstants.kIFrontRightVel, AutoConstants.kDFrontRightVel), 
                new PIDController(AutoConstants.kPBackRightVel, AutoConstants.kIBackRightVel,AutoConstants.kDBackRightVel),
                m_driveTrain::getCurrentWheelSpeeds,
                m_driveTrain::setDriveMotorControllersVolts, // Consumer for the output motor voltages
                m_driveTrain);

    // Reset odometry to the starting pose of the trajectory.
    m_driveTrain.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return mecanumControllerCommand.andThen(() -> m_driveTrain.drive(0, 0, 0, false));
  }
/* 
  public void cameraProcess(){
    m_visionThread =
        new Thread(
            () -> {
              // Get the Axis camera from CameraServer
              AxisCamera camera = CameraServer.addAxisCamera("axis-camera.local");
              // Set the resolution
              camera.setResolution(640, 480);

              // Get a CvSink. This will capture Mats from the camera
              CvSink cvSink = CameraServer.getVideo();
              // Setup a CvSource. This will send images back to the Dashboard
              CvSource outputStream = CameraServer.putVideo("Rectangle", 640, 480);

              // Mats are very memory expensive. Lets reuse this Mat.
              Mat mat = new Mat();

              // This cannot be 'true'. The program will never exit if it is. This
              // lets the robot stop this thread when restarting robot code or
              // deploying.
              while (!Thread.interrupted()) {
                // Tell the CvSink to grab a frame from the camera and put it
                // in the source mat.  If there is an error notify the output.
                if (cvSink.grabFrame(mat) == 0) {
                  // Send the output the error.
                  outputStream.notifyError(cvSink.getError());
                  // skip the rest of the current iteration
                  continue;
                }
                // Put a rectangle on the image
                Imgproc.rectangle(
                    mat, new Point(100, 100), new Point(400, 400), new Scalar(255, 255, 255), 5);
                // Give the output stream a new image to display
                outputStream.putFrame(mat);
              }
            });
    m_visionThread.setDaemon(true);
    m_visionThread.start();
  }
  */
}
