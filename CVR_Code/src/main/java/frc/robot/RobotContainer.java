// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.HomeCommand;
import frc.robot.commands.TrajectoryCommand;
import frc.robot.commands.ArmManualCommand;
import frc.robot.commands.ArmPresetCommand;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.DriveManualCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;

import java.util.LinkedList;
import java.util.List;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
        driveSubsystem.calibrate();
    }

    private void configureBindings() {
        driveSubsystem.setDefaultCommand(
            //new DriveManualCommand(driveSubsystem, joystick));
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
        
        new JoystickButton(joystick, Constants.BalanceBttn)
            .whileTrue(new BalanceCommand(driveSubsystem));
    }
  
    public Command getAutonomousCommand() {
        LinkedList<Pair<Double,Double>> point1 = new LinkedList<Pair<Double,Double>>();
        LinkedList<Pair<Double,Double>> point2 = new LinkedList<Pair<Double,Double>>();
        point1.add(new Pair<Double,Double>(2.2,0.0));

        return new SequentialCommandGroup(new HomeCommand(armSubsystem),
            new ArmPresetCommand(armSubsystem, 49, 32),
            Commands.runOnce(() -> {armSubsystem.setClaw(true);}),
            new HomeCommand(armSubsystem),
            new TrajectoryCommand(driveSubsystem, point1, 0));
    }

    public void setBraking(boolean braking) {
        driveSubsystem.setBraking(braking);
    }
}