package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class SlowDriveCommand extends CommandBase {
    DriveSubsystem driveSubsystem;
    Joystick joystick;

    public SlowDriveCommand(DriveSubsystem driveSubsystem, Joystick joystick) {
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
        if(x > 0) x = (x - 0.15) * 0.25 + 0.25;
        if(y > 0) y = (y - 0.15) * 0.3 + 0.25;

        if(x < 0) x = (x + 0.15) * 0.25 - 0.25;
        if(y < 0) y = (y + 0.15) * 0.3 - 0.25;

        driveSubsystem.arcadeDrive(-y, x);
    }
}
