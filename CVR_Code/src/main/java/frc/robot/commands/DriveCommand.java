package frc.robot.commands;

import frc.robot.MathUtil;

import java.util.function.Function;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase{
    DriveSubsystem driveSubsystem;
    Joystick joystick;

    public DriveCommand(DriveSubsystem driveSubsystem, Joystick joystick) {
        this.driveSubsystem = driveSubsystem;
        this.joystick = joystick;

        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        Function<Double, Double> scaleRot = MathUtil.scaleMagnitude(0.15, 1.0, 0.3, 0.9, 2.0);
        Function<Double, Double> scaleFwd = MathUtil.scaleMagnitude(0.15, 1.0, 0.3, 1.0, 2.0);
        double x = joystick.getX();
        double y = joystick.getY();
        double rot = (Math.abs(x) < 0.15) ? 0 : x;
        double fwd = (Math.abs(y) < 0.15) ? 0 : y;

        rot = scaleRot.apply(x);
        fwd = scaleFwd.apply(y);
        driveSubsystem.arcadeDrive(-fwd, rot);
    }
}
