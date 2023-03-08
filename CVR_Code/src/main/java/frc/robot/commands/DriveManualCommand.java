package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.IOConstants.SmartDashboardBoolean;
import frc.robot.subsystems.DriveSubsystem;

public class DriveManualCommand extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    private final Joystick joystick;
    private final double maxAcceleration = 0.8; // per 1 second
    private final double maxDecceleration = 0.3; // per 1 second

    private final SmartDashboardBoolean goSlow = new SmartDashboardBoolean("Go Slow", false);

    private long prevSpeedTime;
    private double prevSpeed;

    private final double ignoreThreshold = 0.05;
    private final double maxVoltage = 1.0;
    private final double minVoltage = 0.35;  
    private final double slowMaxVoltage = 0.60;

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
        double rotateSpeed = joystick.getX();
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

        double maxVoltage = goSlow.get() ? this.slowMaxVoltage : this.maxVoltage;
        forwardSpeed += change;

        if (forwardSpeed > ignoreThreshold) {
            forwardSpeed = scale(forwardSpeed, ignoreThreshold, 1.0, minVoltage, maxVoltage);
        } else if (forwardSpeed < -ignoreThreshold) {
            forwardSpeed = scale(forwardSpeed, -1.0, -ignoreThreshold, -maxVoltage, -minVoltage);
        } else {
            forwardSpeed = 0;
        }

        if (rotateSpeed > ignoreThreshold) {
            rotateSpeed = scale(rotateSpeed, ignoreThreshold, 1.0, minVoltage, maxVoltage);
        } else if (forwardSpeed < -ignoreThreshold) {
            rotateSpeed = scale(rotateSpeed, -1.0, -ignoreThreshold, -maxVoltage, -minVoltage);
        } else {
            rotateSpeed = 0;
        }

        driveSubsystem.arcadeDrive(forwardSpeed, rotateSpeed);
        prevSpeedTime = System.currentTimeMillis();
        prevSpeed = forwardSpeed;
    }

    // maps [minX, maxX] to [minY, maxY]
    private double scale(double input, double minX, double maxX, double minY, double maxY) {
        return minY + ((maxY - minY) / (maxX - minX)) * (input - minX);
    }
}

