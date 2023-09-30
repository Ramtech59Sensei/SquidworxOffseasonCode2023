// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.TeleOpArm;
import frc.robot.commands.TeleOpDrive;
import frc.robot.commands.VacuumLetGoCMD;
import frc.robot.commands.VacuumSuckCMD;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Vacuum;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Arm m_Arm = new Arm();
  private final Vacuum m_Vacuum = new Vacuum();
  // The driver's controller
  // public static XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  // public static XboxController m_CoController = new XboxController(OIConstants.kOperatorControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    // m_Arm.setDefaultCommand(new VerticalArmCMD(m_Arm, .5));
    //  m_Arm.setDefaultCommand(new VerticalArmCMD(m_Arm, .5));
    // Configure default commands
    m_robotDrive.setDefaultCommand(new TeleOpDrive(m_robotDrive));
    m_Arm.setDefaultCommand(new TeleOpArm(m_Arm));
    // m_robotDrive.setDefaultCommand(
    //         new RunCommand(
    //             () -> m_robotDrive.drive(
    //                 -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
    //                 -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
    //                 -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
    //                 true, false),
    //             m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

        Trajectory forward = 
        TrajectoryGenerator.generateTrajectory(
          new Pose2d(0, 0, new Rotation2d(0)), 
          List.of(new Translation2d(1, 0)), 
          new Pose2d(5, 0, new Rotation2d(0)), config);

        Trajectory bridge = 
        TrajectoryGenerator.generateTrajectory(
          new Pose2d(0, 0, new Rotation2d(0)), 
          List.of(new Translation2d(1, 0)),
          new Pose2d(1.5, 0, new Rotation2d(0)), config);

          Trajectory backandforth = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
           List.of(new Translation2d(-1,0), new Translation2d(2, 0)), 
           new Pose2d(5, 0, new Rotation2d(0)), config);


    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        backandforth,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(backandforth.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, true, false));
  }
}
