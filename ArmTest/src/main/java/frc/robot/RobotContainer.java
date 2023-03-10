// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.LinkedList;

import frc.robot.commands.ArmCommand;
import frc.robot.subsystems.ArmSubsystem;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final ArmSubsystem m_ArmSubsystem = new ArmSubsystem();
  
    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final Joystick joystick = new Joystick(0);

    private PhotonCamera camera = new PhotonCamera("Private");

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        configureBindings();
        camera.setDriverMode(false);
        camera.setPipelineIndex(1);
    }
  
    private void configureBindings() {
        new JoystickButton(joystick, Constants.liftArmBttn)
            .whileTrue(Commands.run(() -> m_ArmSubsystem.raiseArm(0.25)))
            .onFalse(Commands.run(() -> m_ArmSubsystem.resetPulley()));
        new JoystickButton(joystick, Constants.lowerArmBttn)
            .whileTrue(Commands.run(() -> m_ArmSubsystem.raiseArm(-0.25)))
            .onFalse(Commands.run(() -> m_ArmSubsystem.resetPulley()));
        new JoystickButton(joystick, Constants.extendArmBttn)
            .whileTrue(Commands.run(() -> m_ArmSubsystem.extendArm(0.25)))
            .onFalse(Commands.run(() -> m_ArmSubsystem.resetTelescope()));
        new JoystickButton(joystick, Constants.retractArmBttn)
            .whileTrue(Commands.run(() -> m_ArmSubsystem.extendArm(-0.25)))
            .onFalse(Commands.run(() -> m_ArmSubsystem.resetTelescope()));
        new JoystickButton(joystick, Constants.OpenClawBttn)
            .onTrue(Commands.runOnce(() -> m_ArmSubsystem.toggleClaw()));
    }
}
