package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class RampBalanceCommand extends CommandBase {
    DriveSubsystem driveSubsystem;
    long startTime;
    int scaling;
    double prevPitch;
    double currPitch;
    int counter;

    double pitchTolerance = 2;
    double rateTolerance = 6;

    public RampBalanceCommand(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
        scaling = 1;
        prevPitch = driveSubsystem.getPitch();
    }

    @Override
    public void execute() {
        currPitch = driveSubsystem.getPitch();
        int dir;
        if(currPitch > 0) {
            dir = 1;
        } else {
            dir = -1;
        }
        long currTime = System.currentTimeMillis();
        double delta = Math.abs(currPitch - prevPitch);
        if(delta > prevPitch) {
            scaling++;
            startTime = currTime;
        }
        if(delta/currTime*1000 > rateTolerance) {
            return;
        }
        long timeSince = (System.currentTimeMillis() - startTime)/1000;
        double power = dir*((Math.pow(scaling*timeSince-1.3, 7) + 2*Math.pow(scaling*timeSince-1.3, 4) + 0.5)/(4+scaling) + 0.2);
        driveSubsystem.arcadeDrive(power, 0);
    }

    @Override
    public boolean isFinished() {
        if(Math.abs(currPitch) < pitchTolerance) {
            if(counter >= 100) return true;
            else counter++;
        } else {
            counter = 0;
        }
        return false;
    }
}
