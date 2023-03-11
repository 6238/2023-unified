package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

        if(distance > 0) {
            rate = 0.5;
        } else if (distance < 0) {
            rate = -0.5;
        } else {
            rate = 0;
        }

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        this.startPosition = driveSubsystem.getPosition();
    }

    @Override
    public void execute() {
        driveSubsystem.arcadeDrive(rate, 0);
    }

    @Override
    public boolean isFinished() {
        if(distance > 0) {
            return driveSubsystem.getPosition() >= startPosition + distance;
        } else {
            return driveSubsystem.getPosition() <= startPosition + distance;
        }
    }
}
