// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.HomeCommand;
import frc.robot.commands.ArmCommandFactory;
import frc.robot.commands.ArmManualCommand;
import frc.robot.commands.ArmPresetCommand;
import frc.robot.commands.DriveCommandFactory;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
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
    private final DriveCommandFactory driveCommandFactory = new DriveCommandFactory(driveSubsystem, joystick);
    private final ArmCommandFactory armCommandFactory = new ArmCommandFactory(armSubsystem, joystick);
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
        driveSubsystem.setDefaultCommand(driveCommandFactory.getManualDriveCommand());

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
            .onTrue(armCommandFactory.getHomeCommand());

        new JoystickButton(joystick, Constants.ShelfBttn)
            .onTrue(armCommandFactory.getArmPresetCommand(67.7, 35.6));
        
        // Old : Pulley (70.7), Telescope (98.2)
            new JoystickButton(joystick, Constants.GridHighBttn)
            .onTrue(armCommandFactory.getArmPresetCommand(67.5, 97.6));

        new JoystickButton(joystick, Constants.GridLowBttn)
            .onTrue(armCommandFactory.getArmPresetCommand(130, 0));

        // Old : Pulley (64.3), Telescope (18.7)
        new JoystickButton(joystick, Constants.GridMidBttn)
            .onTrue(armCommandFactory.getArmPresetCommand(64.4, 29.2));
        
        new JoystickButton(joystick, Constants.BalanceBttn)
            .whileTrue(driveCommandFactory.getBalanceCommand(0.25, 0.4, 2));

        new JoystickButton(joystick, Constants.SlowBttn)
            .whileTrue(driveCommandFactory.getManualDriveSlowCommand());
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
        // LinkedList<Pair<Double,Double>> point1 = new LinkedList<Pair<Double,Double>>();
        // point1.add(new Pair<Double,Double>(-3.5,0.0));
        return autonomousThree();
        // return new SequentialCommandGroup(new HomeCommand(armSubsystem),
        //     // new ArmPresetCommand(armSubsystem, 66, 36.5),
        //     // Commands.waitSeconds(0.1),
        //     // Commands.runOnce(() -> {armSubsystem.setClaw(true);}),
        //     // Commands.waitSeconds(0.5),
        //     // new HomeCommand(armSubsystem),
        //     driveSubsystem.getTimedDrive(3000, -0.55));
            //new TrajectoryCommand(driveSubsystem, point1, 0));
    }

    private Command autonomousThree() {
        // LinkedList<Pair<Double,Double>> point1 = new LinkedList<Pair<Double,Double>>();
        // point1.add(new Pair<Double,Double>(-3.5,0.0));

        return new SequentialCommandGroup(new HomeCommand(armSubsystem),
            new ArmPresetCommand(armSubsystem, 64.4, 29.2),
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