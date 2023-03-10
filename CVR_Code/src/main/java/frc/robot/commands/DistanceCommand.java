package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DistanceCommand extends CommandBase {
    DriveSubsystem driveSubsystem;
    double distance;
    double startPosition;
    double rate;
    
    public DistanceCommand(DriveSubsystem driveSubsystem, double distance) {
        this.driveSubsystem = driveSubsystem;
        this.distance = distance;
        this.startPosition = driveSubsystem.getPosition();

        if(distance > 0) {
            rate = 0.7;
        } else if (distance < 0) {
            rate = -0.7;
        } else {
            rate = 0;
        }
    }

    @Override
    public void execute() {
        driveSubsystem.arcadeDrive(rate, 0);
    }

    @Override
    public boolean isFinished() {
        return driveSubsystem.getPosition() >= startPosition + distance;
    }

    @Override
    public void end(boolean interrupt) {
        driveSubsystem.arcadeDrive(0, 0);
    }
}
