package frc.robot.commands;

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
        double x = joystick.getX();
        double y = joystick.getY();
        x = (Math.abs(x) < 0.15) ? 0 : x;
        y = (Math.abs(y) < 0.15) ? 0 : y;

        x = Math.abs(x) * x;
        y = Math.abs(y) * y;
        if(x > 0) x = (x - 0.15) * 0.6 + 0.3;
        if(y > 0) y = (y - 0.15) * 0.7 + 0.3;

        if(x < 0) x = (x + 0.15) * 0.6 - 0.3;
        if(y < 0) y = (y + 0.15) * 0.7 - 0.3;

        driveSubsystem.arcadeDrive(-y, x);
    }
}
