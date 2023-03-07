package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveManualCommand extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    private final Joystick joystick;
    private final double maxAcceleration = 0.8; // per 1 second
    private final double maxDecceleration = 0.3; // per 1 second

    private long prevSpeedTime;
    private double prevSpeed;

    public DriveManualCommand(DriveSubsystem driveSubsystem, Joystick joystick) {
        this.joystick = joystick;
        this.driveSubsystem = driveSubsystem;
    }

    @Override
    public void initialize() {
        prevSpeed = joystick.getY();
        prevSpeedTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        long timePassed = (System.currentTimeMillis() - prevSpeedTime) / 1000;
        double forwardSpeed = joystick.getY();
        double change;

        if (prevSpeed > 0) {
            change = MathUtil.clamp(forwardSpeed - prevSpeed,
                -maxDecceleration * timePassed,
                maxAcceleration * timePassed);
        } else {
            change = MathUtil.clamp(forwardSpeed - prevSpeed,
                -maxAcceleration * timePassed,
                maxDecceleration * timePassed); 
        }

        forwardSpeed += change;
        driveSubsystem.arcadeDrive(forwardSpeed, joystick.getX());
        prevSpeedTime = System.currentTimeMillis();
        prevSpeed = forwardSpeed;

    }
}
