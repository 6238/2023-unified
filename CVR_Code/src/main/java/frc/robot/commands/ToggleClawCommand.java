package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;

public class ToggleClawCommand extends SequentialCommandGroup {
    ArmSubsystem armSubsystem;
    long startTime;

    public ToggleClawCommand(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
        BooleanSupplier timeReached = () -> System.currentTimeMillis() == startTime+350;

        addRequirements(armSubsystem);
        addCommands(
            Commands.runOnce(() -> {
                armSubsystem.toggleClaw();
            }),
            Commands.waitSeconds(0.1),
            Commands.runOnce(() -> {
                startTime = System.currentTimeMillis();
            }),
            Commands.run(() -> {
                armSubsystem.raiseArm(1);
            }).until(timeReached),
            new HomeCommand(armSubsystem)
        );
    }
}
