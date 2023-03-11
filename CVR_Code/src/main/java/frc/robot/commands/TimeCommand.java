package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TimeCommand extends CommandBase {
    DriveSubsystem driveSubsystem;
    double time;
    double startPosition;
    long startTime;
    int dir;
    
    public TimeCommand(DriveSubsystem driveSubsystem, double time, int dir) {
        this.driveSubsystem = driveSubsystem;
        this.time = time;
        this.dir = dir;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        this.startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        driveSubsystem.arcadeDrive(dir*0.7, 0);
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() >= startTime + time;
        // if(distance > 0) {
        //     return driveSubsystem.getPosition() >= startPosition + distance;
        // } else {
        //     return driveSubsystem.getPosition() <= startPosition + distance;
        // }
    }

    @Override
    public void end(boolean interrupt) {
        System.out.println("ended");
        driveSubsystem.arcadeDrive(0, 0);
    }
}
