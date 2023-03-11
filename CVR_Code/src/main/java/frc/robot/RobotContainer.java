// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.HomeCommand;
import frc.robot.commands.SlowDriveCommand;
import frc.robot.commands.DistanceCommand;
import frc.robot.commands.ToggleClawCommand;
import frc.robot.commands.TrajectoryCommand;
import frc.robot.commands.ArmManualCommand;
import frc.robot.commands.ArmPresetCommand;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;

import java.util.LinkedList;
import java.util.List;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private SimpleWidget autoSelector;

    public RobotContainer() {
        // SendableChooser<Integer> autoModeSelection = new SendableChooser<Integer>();
        // autoModeSelection.addOption("Balance", 0);
        // autoModeSelection.addOption("No Balance", 1);
        // autoSelector = Shuffleboard.getTab("Control")
        //     .add("Auto Mode", 0)
        //     .withWidget(BuiltInWidgets.kSplitButtonChooser);
        configureBindings();
        driveSubsystem.calibrate();
    }

    private void configureBindings() {
        driveSubsystem.setDefaultCommand(new DriveCommand(driveSubsystem, joystick));

        new JoystickButton(joystick, Constants.raiseArmBttn)
            .whileTrue(new ArmManualCommand(armSubsystem, joystick));
  
        new JoystickButton(joystick, Constants.lowerArmBttn)
            .whileTrue(new ArmManualCommand(armSubsystem, joystick));
  
        new JoystickButton(joystick, Constants.extendArmBttn)
            .whileTrue(new ArmManualCommand(armSubsystem, joystick));
  
        new JoystickButton(joystick, Constants.retractArmBttn)
            .whileTrue(new ArmManualCommand(armSubsystem, joystick));
  
        new JoystickButton(joystick, Constants.OpenClawBttn)
            .onTrue(Commands.runOnce(() -> armSubsystem.toggleClaw()));

        new JoystickButton(joystick, Constants.HomeBttn)
            .onTrue(new HomeCommand(armSubsystem));

        new JoystickButton(joystick, Constants.ShelfBttn)
            .onTrue(new ArmPresetCommand(armSubsystem, 73.8, 36.8));
        
            new JoystickButton(joystick, Constants.GridHighBttn)
            .onTrue(new ArmPresetCommand(armSubsystem, 72.5, 97.7));

        new JoystickButton(joystick, Constants.GridLowBttn)
            .onTrue(new ArmPresetCommand(armSubsystem, 130, 0));

        new JoystickButton(joystick, Constants.GridMidBttn)
            .onTrue(new ArmPresetCommand(armSubsystem, 64.3, 18.7));
        
        new JoystickButton(joystick, Constants.BalanceBttn)
            .whileTrue(new BalanceCommand(driveSubsystem));

        new JoystickButton(joystick, Constants.SlowBttn)
            .whileTrue(new SlowDriveCommand(driveSubsystem, joystick));
    }
  
    public Command getAutonomousCommand() {
        int mode = (int)SmartDashboard.getNumber("Auto Mode", 2); //autoSelector.getEntry().getInteger(0);
        switch(mode) {
            case 0:
                return autonomousOne();
            case 1:
                return autonomousTwo();
            case 2:
                return autonomousThree();
            default:
                return autonomousOne();
        }
    }

    private Command autonomousOne() {
        // LinkedList<Pair<Double,Double>> point1 = new LinkedList<Pair<Double,Double>>();
        // point1.add(new Pair<Double,Double>(-2.2,0.0));

        return new SequentialCommandGroup(new HomeCommand(armSubsystem),
            new ArmPresetCommand(armSubsystem, 66, 36.5),
            Commands.waitSeconds(0.1),
            Commands.runOnce(() -> {armSubsystem.setClaw(true);}),
            Commands.waitSeconds(0.5),
            new HomeCommand(armSubsystem),
            new DistanceCommand(driveSubsystem, -2.2),
            new BalanceCommand(driveSubsystem));
            // new TrajectoryCommand(driveSubsystem, point1, 0));
    }

    private Command autonomousTwo() {
        // LinkedList<Pair<Double,Double>> point1 = new LinkedList<Pair<Double,Double>>();
        // point1.add(new Pair<Double,Double>(-3.5,0.0));

        return new SequentialCommandGroup(new HomeCommand(armSubsystem),
            new ArmPresetCommand(armSubsystem, 66, 36.5),
            Commands.waitSeconds(0.1),
            Commands.runOnce(() -> {armSubsystem.setClaw(true);}),
            Commands.waitSeconds(0.5),
            new HomeCommand(armSubsystem),
            new DistanceCommand(driveSubsystem, -3.5));
            //new TrajectoryCommand(driveSubsystem, point1, 0));
    }

    private Command autonomousThree() {
        // LinkedList<Pair<Double,Double>> point1 = new LinkedList<Pair<Double,Double>>();
        // point1.add(new Pair<Double,Double>(-3.5,0.0));

        return new SequentialCommandGroup(new HomeCommand(armSubsystem),
            new ArmPresetCommand(armSubsystem, 66, 36.5),
            Commands.waitSeconds(0.1),
            Commands.runOnce(() -> {armSubsystem.setClaw(true);}),
            Commands.waitSeconds(0.5),
            new HomeCommand(armSubsystem));
            //new TrajectoryCommand(driveSubsystem, point1, 0));
    }

    public void setBraking(boolean braking) {
        driveSubsystem.setBraking(braking);
    }

    public void reset() {
        armSubsystem.setClaw(false);
        new HomeCommand(armSubsystem).schedule();
    }
}