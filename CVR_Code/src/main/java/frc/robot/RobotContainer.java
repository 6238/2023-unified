// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.HomeCommand;
import frc.robot.commands.ArmPresetCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;

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
        driveSubsystem.setDefaultCommand(
            Commands.run(() -> driveSubsystem.arcadeDrive(-joystick.getY(), joystick.getX()), driveSubsystem));

        new JoystickButton(joystick, Constants.raiseArmBttn)
            .whileTrue(Commands.run(() -> armSubsystem.raiseArm(1)))
            .onFalse(Commands.run(() -> armSubsystem.resetPulley()));
  
        new JoystickButton(joystick, Constants.lowerArmBttn)
            .whileTrue(Commands.run(() -> armSubsystem.raiseArm(-1)))
            .onFalse(Commands.run(() -> armSubsystem.resetPulley()));
  
        new JoystickButton(joystick, Constants.extendArmBttn)
            .whileTrue(Commands.run(() -> armSubsystem.extendTelescope(1)))
            .onFalse(Commands.run(() -> armSubsystem.resetTelescope()));
  
        new JoystickButton(joystick, Constants.retractArmBttn)
             .whileTrue(Commands.run(() -> armSubsystem.extendTelescope(-1)))
             .onFalse(Commands.run(() -> armSubsystem.resetTelescope()));
  
        new JoystickButton(joystick, Constants.OpenClawBttn)
              .whileTrue(Commands.runOnce(() -> armSubsystem.toggleClaw()));

        new JoystickButton(joystick, Constants.HomeBttn)
            .onTrue(new HomeCommand(armSubsystem));

        new JoystickButton(joystick, Constants.ShelfBttn)
        
            .onTrue(new ArmPresetCommand(armSubsystem, 81,77));
        
            new JoystickButton(joystick, Constants.GridHighBttn)
            .onTrue(new ArmPresetCommand(armSubsystem, 60,97));

        new JoystickButton(joystick, Constants.GridLowBttn)
            .onTrue(new ArmPresetCommand(armSubsystem, 130,0));

        new JoystickButton(joystick, Constants.GridMidBttn)
            .onTrue(new ArmPresetCommand(armSubsystem, 49,32));
        

    }
  
    public Command getAutonomousCommand() {
        return null;
    }
}