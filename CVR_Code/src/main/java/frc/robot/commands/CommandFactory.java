package frc.robot.commands;

import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.MathUtil;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class CommandFactory {
    private final DriveSubsystem driveSubsystem;
    private final ArmSubsystem armSubsystem;

    public CommandFactory(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.armSubsystem = armSubsystem;
    }

    public Command getTimedDrive(long timeMS, double power) {
        final class Timer {
            long setPoint;
            public Timer(long timeMS) { this.setPoint = System.currentTimeMillis() + timeMS; }
            public boolean isFinished() { return System.currentTimeMillis() >= setPoint; }
        };
        
        Timer timer = new Timer(timeMS);
        return Commands.run(() -> driveSubsystem.arcadeDrive(-power, 0), driveSubsystem)
            .until(timer::isFinished).andThen(Commands.runOnce(()-> driveSubsystem.arcadeDrive(0, 0)));
    }

    public Command getBalanceCommand(double minVoltage, double maxVoltage, double delayThresholdDegPerS) {
		final double maxPitch = 20.0;
		MathUtil.SpeedGetter speedGetter = new MathUtil.SpeedGetter(() -> { return driveSubsystem.getPitch(); });
        Function<Double, Double> scale = MathUtil.scaleMagnitude(0.0, maxPitch, minVoltage, maxVoltage, 1.5);

		Supplier<Double> fwd = () -> {
			return speedGetter.get() > delayThresholdDegPerS ? 0 : scale.apply(driveSubsystem.getPitch());
		};

		return Commands.run(() -> driveSubsystem.arcadeDrive(fwd.get(), 0), driveSubsystem);
	}
}
