package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class BalanceCommand extends CommandBase {
    private final double angleErrorTolerance = 2;
    private final double maxDegreePerSecond = 1;

    private final double minVoltage = 0.35;
    private final double maxVoltage = 0.6;
    // 0 < minVoltage < maxVoltage < 1.0

    private final DriveSubsystem driveSubsystem;  
    private double prevPitch;
    private long timeAtPrevPitch;
    private long timeSinceStart;
    // private boolean driving = true;
    // private long timeAtSwitch;

    public BalanceCommand(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        prevPitch = driveSubsystem.getPitch();
        timeAtPrevPitch = System.currentTimeMillis();
        timeSinceStart = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        double pitch = driveSubsystem.getPitch();
        double fwd;
        double timeScale = 0;
        timeScale = (System.currentTimeMillis() - timeSinceStart)/10000;
        if(pitch > 0) {
            fwd = -(Math.sin(pitch*Math.PI/180)) * (maxVoltage - minVoltage - timeScale) - minVoltage;
        } else {
            fwd = -(Math.sin(pitch*Math.PI/180)) * (maxVoltage - minVoltage - timeScale) + minVoltage;
        }
        System.out.println(fwd);
        //System.out.println("Original: " + -(Math.sin(pitch)) * (maxVoltage - minVoltage) + ", Scaled: " + fwd);
        // maps [0, maxPitch] to [minVoltage, maxVoltage]
        driveSubsystem.arcadeDrive(fwd, 0);
    }

    @Override
    public boolean isFinished() {
        double newPitch = driveSubsystem.getPitch();
        long newTime = System.currentTimeMillis();
        if (1000 * Math.abs((newPitch - prevPitch)) / (newTime - timeAtPrevPitch) <= maxDegreePerSecond
            // check error derivative value
            && Math.abs(newPitch) <= angleErrorTolerance
            // check position value
            ) {
            return true;
        }
        prevPitch = newPitch;
        timeAtPrevPitch = newTime;
        return false;
    }
}
