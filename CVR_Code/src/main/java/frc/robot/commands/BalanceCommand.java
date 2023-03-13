package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class BalanceCommand extends CommandBase {
    private final double angleErrorTolerance = 2;
    private final double maxDegreePerSecond = 2;
    private final double maxPitch = 20.0;

    private double minVoltage = 0.3;
    private double maxVoltage = 0.45;
    // 0 < minVoltage < maxVoltage < 1.0

    private final DriveSubsystem driveSubsystem;  
    private double prevPitch;
    private long timeAtPrevPitch;
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
    }

    @Override
    public void execute() {
        double pitch = driveSubsystem.getPitch();
        double fwd;
        long time = System.currentTimeMillis();
        if(1000*Math.abs(pitch - prevPitch) / (time - timeAtPrevPitch) > maxDegreePerSecond) {
            driveSubsystem.arcadeDrive(0, 0);
            return;
        } 
        prevPitch = pitch;
        timeAtPrevPitch = time;

        if(pitch > 0) {
            fwd = Math.pow(pitch / maxPitch, 1.5) * (maxVoltage - minVoltage) - minVoltage;
        } else {
            fwd = -Math.pow(-pitch / maxPitch, 1.5) * (maxVoltage - minVoltage) + minVoltage;
        }

        driveSubsystem.arcadeDrive(fwd, 0);
    }

    // @Override
    // public boolean isFinished() {
    //     double newPitch = driveSubsystem.getPitch();
    //     long newTime = System.currentTimeMillis();
    //     if (1000 * Math.abs((newPitch - prevPitch)) / (newTime - timeAtPrevPitch) <= maxDegreePerSecond
    //         // check error derivative value
    //         && Math.abs(newPitch) <= angleErrorTolerance
    //         // check position value
    //         ) {
    //         return true;
    //     }
    //     prevPitch = newPitch;
    //     timeAtPrevPitch = newTime;
    //     return false;
    // }
}
