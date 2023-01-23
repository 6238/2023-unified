package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class BalanceCommand extends CommandBase {
    private final double angleErrorTolerance = 2;
    private final double maxDegreePerSecond = 1.5;

    private final double maxPitch = 20.0;
    private final double minVoltage = 0.25;
    private final double maxVoltage = 0.50;
    // 0 < minVoltage < maxVoltage < 1.0

    private long startTime;

    private final DriveSubsystem driveSubsystem;  
    private double prevPitch;
    private long timeAtPrevPitch;

    public BalanceCommand(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);

        prevPitch = driveSubsystem.getPitch();
        timeAtPrevPitch = System.currentTimeMillis();

        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        double newPitch = driveSubsystem.getPitch();
        long newTime = System.currentTimeMillis();
        if (1000 * Math.abs((newPitch - prevPitch))
            / (newTime - timeAtPrevPitch) > maxDegreePerSecond) {
            return;
        }

        double pitch = driveSubsystem.getPitch();
        double fwd;
        if (pitch > 0) {
            fwd = -Math.pow(pitch / maxPitch, 1.5) * (maxVoltage - minVoltage) - minVoltage;
        } else {
            fwd = Math.pow(-pitch / maxPitch, 1.5) * (maxVoltage - minVoltage) + minVoltage;
        }

        // maps [0, maxPitch] to [minVoltage, maxVoltage]
        driveSubsystem.arcadeDrive(fwd, 0);
    }

    @Override
    public boolean isFinished() {
        double newPitch = driveSubsystem.getPitch();
        long newTime = System.currentTimeMillis();
        
        // if (1000 * Math.abs((newPitch - prevPitch)) / (newTime - timeAtPrevPitch) <= maxDegreePerSecond
        //     // check error derivative value
        //     && Math.abs(newPitch) <= angleErrorTolerance
        //     // check position value
        //     ) {
        //     return true;
        // }
        // 
        prevPitch = newPitch;
        timeAtPrevPitch = newTime;
        
        return false;
    }
}
