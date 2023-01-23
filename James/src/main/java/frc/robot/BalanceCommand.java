package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class BalanceCommand extends CommandBase {
    private final double maxDegreePerSecond = 1.5;

    private final double maxPitch = 20.0;
    private final double minVoltage = 0.25;
    private final double maxVoltage = 0.50;
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
        long time = System.currentTimeMillis();
        if (1000 * Math.abs((pitch - prevPitch))
            / (time - timeAtPrevPitch) > maxDegreePerSecond) {
            return;
        }

        prevPitch = pitch;
        timeAtPrevPitch = time;

        double fwd;
        if (pitch > 0) {
            fwd = -Math.pow(pitch / maxPitch, 1.5) * (maxVoltage - minVoltage) - minVoltage;
        } else {
            fwd = Math.pow(-pitch / maxPitch, 1.5) * (maxVoltage - minVoltage) + minVoltage;
        }
        // maps [0, maxPitch] to [minVoltage, maxVoltage]
        driveSubsystem.arcadeDrive(fwd, 0);
    }
}
