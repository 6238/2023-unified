// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.BalanceCommand;
import frc.robot.commands.ObjectCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.commands.TrajectoryCommand;

import java.util.List;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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

    private PhotonCamera camera = new PhotonCamera("Private");
      
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        configureBindings();
        camera.setPipelineIndex(1);
        camera.setDriverMode(true);
        m_robotDrive.setDefaultCommand(
            Commands.run(() -> m_robotDrive.arcadeDrive(-joystick.getY(), joystick.getX()), m_robotDrive)
        );
    }
  
    private void configureBindings() {
        new JoystickButton(joystick, Constants.BalanceRobotBttn)
            .whileTrue(new BalanceCommand(m_robotDrive));
        new JoystickButton(joystick, Constants.ConeButton)
            .whileTrue(new ObjectCommand(m_robotDrive, camera, 0));
    }
  
    public Command getAutonomousCommand() {
        double distance = SmartDashboard.getNumber("Distance To Travel", 1);
        Pair[] points = {new Pair<Double,Double>(distance,0.0)};
        return new TrajectoryCommand(m_robotDrive, points, 0.0)
            .andThen(() -> m_robotDrive.tankDriveVolts(0,0));
    }

    public boolean isBraking() {
        return m_robotDrive.isBraking();
    }

    public void setBraking(boolean braking) {
        m_robotDrive.setBraking(braking);
    }

    public PhotonCamera getCamera() {
        return camera;
    }
}
