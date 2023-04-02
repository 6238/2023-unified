// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.HomeCommand;
import frc.robot.commands.SlowDriveCommand;
import frc.robot.commands.ArmManualCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
            .onTrue(Commands.runOnce(() -> armSubsystem.activateSetpointMode(28.33, 7.64), //updated with new values
            armSubsystem));
        
        // Old : Pulley (70.7), Telescope (98.2)
        new JoystickButton(joystick, Constants.GridHighBttn)
            .onTrue(Commands.runOnce(() -> armSubsystem.activateSetpointMode(34.24, 39.81),
            armSubsystem));

        new JoystickButton(joystick, Constants.GridLowBttn)
            .onTrue(Commands.runOnce(() -> armSubsystem.activateSetpointMode(130  / 3, 0),
            armSubsystem));

        // Old : Pulley (64.3), Telescope (18.7)
        new JoystickButton(joystick, Constants.GridMidBttn)
            .onTrue(Commands.runOnce(() -> armSubsystem.activateSetpointMode(30.12, 7.24),
            armSubsystem));
        
        new JoystickButton(joystick, Constants.BalanceBttn)
            .whileTrue(driveSubsystem.getBalanceCommand(0.25, 0.4, 2, 2));

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
        return autonomousTwo();
        /*
        return new SequentialCommandGroup(new HomeCommand(armSubsystem),
            new ArmPresetCommand(armSubsystem, 66, 36.5),
            Commands.waitSeconds(0.1),
            Commands.runOnce(() -> {armSubsystem.setClaw(true);}),
            Commands.waitSeconds(0.5),
            new HomeCommand(armSubsystem),
            driveSubsystem.getTimedDrive(1500, -0.65),
            new BalanceCommand(driveSubsystem));
            // new TrajectoryCommand(driveSubsystem, point1, 0));
        */
    }

    private Command autonomousTwo() {
        return new SequentialCommandGroup(new HomeCommand(armSubsystem),
            Commands.runOnce(() -> armSubsystem.activateSetpointMode(19.8, 36.5),
            armSubsystem),
            Commands.waitSeconds(0.1),
            Commands.runOnce(() -> {armSubsystem.setClaw(true);}),
            Commands.waitSeconds(0.5),
            new HomeCommand(armSubsystem),
            driveSubsystem.getTimedDrive(2000, -0.5),
            driveSubsystem.getBalanceCommand(0.25, 0.4, 2, 2));
    }

    private Command autonomousThree() {
        return new SequentialCommandGroup(new HomeCommand(armSubsystem),
            Commands.runOnce(() -> armSubsystem.activateSetpointMode(64.4, 29.2),
            armSubsystem),
            Commands.waitSeconds(0.1),
            Commands.runOnce(() -> {armSubsystem.setClaw(true);}),
            Commands.waitSeconds(0.5),
            new HomeCommand(armSubsystem));
    }

    public void setBraking(boolean braking) {
        driveSubsystem.setBraking(braking);
    }

    public void reset() {
        armSubsystem.setClaw(false);
        new HomeCommand(armSubsystem).schedule();
    }
}