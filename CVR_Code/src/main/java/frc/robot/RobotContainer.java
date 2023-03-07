// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DriveManualCommand;
import frc.robot.commands.HomeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;

import java.sql.PreparedStatement;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private final ArmSubsystem armSubsystem = new ArmSubsystem();
    private final Joystick joystick = new Joystick(0);
    private final DriveSubsystem driveSubsystem = new DriveSubsystem();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        driveSubsystem.setDefaultCommand(new DriveManualCommand(driveSubsystem, joystick));

        new JoystickButton(joystick, Constants.raiseArmBttn)
            .whileTrue(Commands.run(() -> armSubsystem.raiseArm(0.25)))
            .onFalse(Commands.run(() -> armSubsystem.resetPulley()));
  
        new JoystickButton(joystick, Constants.lowerArmBttn)
            .whileTrue(Commands.run(() -> armSubsystem.raiseArm(-0.25)))
            .onFalse(Commands.run(() -> armSubsystem.resetPulley()));
  
        new JoystickButton(joystick, Constants.extendArmBttn)
            .whileTrue(Commands.run(() -> armSubsystem.extendTelescope(0.25)))
            .onFalse(Commands.run(() -> armSubsystem.resetTelescope()));
  
        new JoystickButton(joystick, Constants.retractArmBttn)
             .whileTrue(Commands.run(() -> armSubsystem.extendTelescope(-0.25)))
             .onFalse(Commands.run(() -> armSubsystem.resetTelescope()));
  
        new JoystickButton(joystick, Constants.OpenClawBttn)
              .whileTrue(Commands.runOnce(() -> armSubsystem.toggleClaw()));

        new JoystickButton(joystick, Constants.HomeBttn)
            .onTrue(new HomeCommand(armSubsystem));
    
    }
  
    public Command getAutonomousCommand() {
        return null;
    }
}
