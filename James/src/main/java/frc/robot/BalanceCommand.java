package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class BalanceCommand extends CommandBase {
    private final double angleErrorTolerance = 2;
    private final double maxDegreePerSecond = 1;

    private final double maxPitch = 16.0;
    private final double minVoltage = 0.40;
    private final double maxVoltage = 1.00;
    // 0 < minVoltage < maxVoltage < 1.0

    private final DriveSubsystem driveSubsystem;  
    private double prevPitch;
    private long timeAtPrevPitch;

    public BalanceCommand(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);

        prevPitch = driveSubsystem.getPitch();
        timeAtPrevPitch = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        double pitch = driveSubsystem.getPitch();
        double fwd = -(pitch / maxPitch) * (maxVoltage - minVoltage) + minVoltage;
        // maps [0, maxPitch] to [minVoltage, maxVoltage]
        driveSubsystem.arcadeDrive(fwd, 0);
    }

    @Override
    public boolean isFinished() {
        double newPitch = driveSubsystem.getPitch();
        long newTime = System.currentTimeMillis();
        if (1000 * Math.abs((newPitch - prevPitch)) / (newTime - timeAtPrevPitch) <= maxDegreePerSecond
            // check error derivative value
            && newPitch <= angleErrorTolerance
            // check position value
            ) {
            return true;
        }
        prevPitch = newPitch;
        timeAtPrevPitch = newTime;
        return false;
    }
}
