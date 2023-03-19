package frc.robot.commands;


import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.MathUtil;
import frc.robot.subsystems.ArmSubsystem;


public class ArmCommandFactory {
    private final ArmSubsystem armSubsystem;
    private final Joystick joystick;


    public ArmCommandFactory(ArmSubsystem armSubsystem, Joystick joystick) {
        this.armSubsystem = armSubsystem;
        this.joystick = joystick;
    }

    public Command getArmPresetCommand(double pulleyPostionTarget, double telescopePositionTarget) {
        BangBangController armController = new BangBangController(3);
        BangBangController telescopeController = new BangBangController(3);
        armController.setSetpoint(pulleyPostionTarget);
        telescopeController.setSetpoint(telescopePositionTarget);

        Supplier<Integer> telescopeDirection
            = () -> -Double.compare(telescopePositionTarget, armSubsystem.getTelescopePosition()) ;
        Supplier<Integer> pulleyDirection
            = () -> Double.compare(pulleyPostionTarget, armSubsystem.getPulleyPosition());
        
        return Commands.run(() -> {
            armSubsystem.raiseArm(pulleyDirection.get()
                * armController.calculate(armSubsystem.getPulleyPosition()));
            armSubsystem.extendTelescope(telescopeDirection.get()
                * telescopeController.calculate(armSubsystem.getTelescopePosition()));
        }, armSubsystem)
        .until(() ->
            armController.atSetpoint()
            && telescopeController.atSetpoint()
        );
    }

    public Command getHomeCommand() {
        MathUtil.Timer timer = new MathUtil.Timer();
        return Commands.run(() -> {
                armSubsystem.raiseArm(1);
                armSubsystem.extendTelescope(-1);
            }, armSubsystem)
            .beforeStarting(() ->
                timer.start(500), armSubsystem)
            .until(() ->
                timer.isFinished()
                && armSubsystem.isPulleyStalled()
                && armSubsystem.isTelescopeStalled())
            .andThen(() -> {
                armSubsystem.setStationary();
                armSubsystem.resetHome();
            }, armSubsystem);
    }

    public Command ToggleClawCommand() {
        MathUtil.Timer timer = new MathUtil.Timer();

        return Commands.sequence(
            Commands.runOnce(() ->
                armSubsystem.toggleClaw(), armSubsystem),
            Commands.waitSeconds(0.1),
            Commands.runOnce(() -> timer.start(350), armSubsystem),
            Commands.run(() -> {
                armSubsystem.raiseArm(1);
            }, armSubsystem)
            .until(timer::isFinished),
            getHomeCommand());
    }
}