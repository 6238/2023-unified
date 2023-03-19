package frc.robot.commands;

import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.MathUtil;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommandFactory {
    private final DriveSubsystem driveSubsystem;
    private final Joystick joystick;

    public DriveCommandFactory(DriveSubsystem driveSubsystem, Joystick joystick) {
        this.driveSubsystem = driveSubsystem;
        this.joystick = joystick;
    }

    public Command getManualDriveCommand() {
        Function<Double, Double> scaleRot = MathUtil.scaleMagnitude(0.15, 1.0, 0.3, 0.9, 2.0);
        Function<Double, Double> scaleFwd = MathUtil.scaleMagnitude(0.15, 1.0, 0.3, 1.0, 2.0);
        return Commands.run(() -> driveSubsystem.arcadeDrive(scaleFwd.apply(-joystick.getY()), scaleRot.apply(joystick.getX())), driveSubsystem);
    }

    public Command getManualDriveSlowCommand() {
        Function<Double, Double> scaleRot = MathUtil.scaleMagnitude(0.15, 1.0, 0.25, 0.5, 2.0);
        Function<Double, Double> scaleFwd = MathUtil.scaleMagnitude(0.15, 1.0, 0.25, 0.55, 2.0);
        return Commands.run(() -> driveSubsystem.arcadeDrive(scaleFwd.apply(-joystick.getY()), scaleRot.apply(joystick.getX())), driveSubsystem);
    }


    public Command getTimedDrive(long timeMS, double power) {
        MathUtil.Timer timer = new MathUtil.Timer(timeMS);
        return Commands.run(() -> driveSubsystem.arcadeDrive(-power, 0), driveSubsystem)
            .until(timer::isFinished).andThen(Commands.runOnce(()-> driveSubsystem.arcadeDrive(0, 0), driveSubsystem));
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
