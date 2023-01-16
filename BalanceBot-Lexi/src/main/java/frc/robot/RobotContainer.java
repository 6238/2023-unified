// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.DriveSubsystem;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  
    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final Joystick joystick = new Joystick(0);
  
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        configureBindings();
        m_robotDrive.setDefaultCommand(
            Commands.run(() -> m_robotDrive.arcadeDrive(-joystick.getY(), joystick.getX()), m_robotDrive)
        );
    }
  
    private void configureBindings() {}
  
    public Command getAutonomousCommand() {
        var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                Constants.ksVolts,
                Constants.kvVoltSecondsPerMeter,
                Constants.kaVoltSecondsSquaredPerMeter),
            Constants.kDriveKinematics,
            10);

        TrajectoryConfig config =
        new TrajectoryConfig(
            Constants.kMaxSpeedMetersPerSecond,
            Constants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

       // double distanceSetPoint = SmartDashboard.getNumber("Distance Set Point", 1);
       // SmartDashboard.putNumber("Distance Set Point", distanceSetPoint);
       // distanceSetPoint = SmartDashboard.getNumber("Distance Set Point", 1);

        Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(),//new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 1 meter straight ahead of where we started, facing forward
            new Pose2d(2, 0, new Rotation2d(0)),
            // Pass config
            config);
        
        m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());
        System.out.println("Initial X Position: " + m_robotDrive.getPose().getX());
        System.out.println("Initial Y Position: " + m_robotDrive.getPose().getY());
        System.out.println("Initial Angle Position: " + m_robotDrive.getPose().getRotation().getDegrees());

        RamseteCommand ramseteCommand = new RamseteCommand(
            exampleTrajectory,
            m_robotDrive::getPose,
            new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
            new SimpleMotorFeedforward(
                Constants.ksVolts,
                Constants.kvVoltSecondsPerMeter,
                Constants.kaVoltSecondsSquaredPerMeter),
            Constants.kDriveKinematics,
            m_robotDrive::getWheelSpeeds,
            new PIDController(Constants.kPDriveVel, 0, 0),
            new PIDController(Constants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            m_robotDrive::tankDriveVolts,
            m_robotDrive);
        
         return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0,0));
       // return Commands.run(() -> m_robotDrive.arcadeDrive(0.8, 0), m_robotDrive);
    }
}
