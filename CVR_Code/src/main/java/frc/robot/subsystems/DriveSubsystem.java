// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;

import frc.robot.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
	private final WPI_TalonFX talonLeftLeader = new WPI_TalonFX(Constants.LEFT_LEADER_ID);
    private final WPI_TalonFX talonLeftFollowerOne = new WPI_TalonFX(Constants.LEFT_FOLLOWER_ID_ONE);
    private final WPI_TalonFX talonLeftFollowerTwo = new WPI_TalonFX(Constants.LEFT_FOLLOWER_ID_TWO);
    private final WPI_TalonFX talonRightLeader = new WPI_TalonFX(Constants.RIGHT_LEADER_ID);
    private final WPI_TalonFX talonRightFollowerOne = new WPI_TalonFX(Constants.RIGHT_FOLLOWER_ID_ONE);
    private final WPI_TalonFX talonRightFollowerTwo = new WPI_TalonFX(Constants.RIGHT_FOLLOWER_ID_TWO);
	private final DifferentialDrive robotDrive = new DifferentialDrive(talonLeftLeader, talonRightLeader);

	private double leftEncoderOffset = talonLeftLeader.getSelectedSensorPosition();
	private double rightEncoderOffset = talonRightLeader.getSelectedSensorPosition();

	private final AHRS ahrs = new AHRS(SPI.Port.kMXP);

	private final DifferentialDriveOdometry m_odometry;

	private int counter = 0;

	public DriveSubsystem() {
		talonLeftLeader.configAllSettings(new TalonFXConfiguration());
		talonLeftFollowerOne.configAllSettings(new TalonFXConfiguration());
		talonLeftFollowerTwo.configAllSettings(new TalonFXConfiguration());
		talonRightLeader.configAllSettings(new TalonFXConfiguration());
		talonRightFollowerOne.configAllSettings(new TalonFXConfiguration());
		talonRightFollowerTwo.configAllSettings(new TalonFXConfiguration());

		talonLeftLeader.configOpenloopRamp(Constants.rampRate);
		talonLeftFollowerOne.configOpenloopRamp(Constants.rampRate);
		talonLeftFollowerTwo.configOpenloopRamp(Constants.rampRate);
		talonRightLeader.configOpenloopRamp(Constants.rampRate);
		talonRightFollowerOne.configOpenloopRamp(Constants.rampRate);
		talonRightFollowerTwo.configOpenloopRamp(Constants.rampRate);

    talonLeftFollowerOne.follow(talonLeftLeader);
    talonLeftFollowerTwo.follow(talonLeftLeader);
    talonRightFollowerOne.follow(talonRightLeader);
    talonRightFollowerTwo.follow(talonRightLeader);
    
    talonRightLeader.setInverted(true);
    talonRightFollowerOne.setInverted(true);
    talonRightFollowerTwo.setInverted(true);

    talonLeftLeader.setNeutralMode(NeutralMode.Brake);
    talonLeftFollowerOne.setNeutralMode(NeutralMode.Brake);
    talonLeftFollowerTwo.setNeutralMode(NeutralMode.Brake);
    talonRightLeader.setNeutralMode(NeutralMode.Brake);
    talonRightFollowerOne.setNeutralMode(NeutralMode.Brake);
    talonRightFollowerTwo.setNeutralMode(NeutralMode.Brake);

		resetEncoders();
		zeroGyroAngle();

		SmartDashboard.putBoolean("Brakes", true);
		setBraking(true);

		m_odometry = new DifferentialDriveOdometry(ahrs.getRotation2d(), nativeUnitsToDistanceMeters(talonLeftLeader.getSelectedSensorPosition()), nativeUnitsToDistanceMeters(talonRightLeader.getSelectedSensorPosition()));
	}

	@Override
	public void periodic() {
		m_odometry.update(ahrs.getRotation2d(), nativeUnitsToDistanceMeters(getLeftEncoder()), nativeUnitsToDistanceMeters(getRightEncoder()));
		SmartDashboard.putNumber("X Position", Math.floor(getPose().getX()*1000)/1000);
    SmartDashboard.putNumber("Y Position", Math.floor(getPose().getY()*1000)/1000);
    SmartDashboard.putNumber("Angle Position", Math.floor(getPose().getRotation().getDegrees()*1000)/1000);
		SmartDashboard.putNumber("X Position Graph", Math.floor(getPose().getX()*1000)/1000);
		SmartDashboard.putNumber("Y Position Graph", Math.floor(getPose().getY()*1000)/1000);
    SmartDashboard.putNumber("Angle Position Graph", Math.floor(getPose().getRotation().getDegrees()*1000)/1000);
		SmartDashboard.putNumber("Pitch Angle", getPitch());
	}

	public void arcadeDrive(double fwd, double rot) {
		robotDrive.arcadeDrive(-fwd, -rot);
	}

	private double nativeUnitsToDistanceMeters(double sensorCounts){
      double motorRotations = sensorCounts / Constants.kCountsPerRev;
      double wheelRotations = motorRotations / Constants.kGearRatio;
      double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(Constants.kWheelRadiusInches));

      return positionMeters * 2.5106;
  }

	public DifferentialDriveWheelSpeeds getWheelSpeeds() {
		return new DifferentialDriveWheelSpeeds(nativeUnitsToDistanceMeters(talonLeftLeader.getSelectedSensorVelocity()), nativeUnitsToDistanceMeters(talonRightLeader.getSelectedSensorVelocity()));
	}

	public double getPosition() {
		double position = nativeUnitsToDistanceMeters(talonLeftLeader.getSelectedSensorPosition() / 2 + talonRightLeader.getSelectedSensorPosition() / 2);
        return position;
	}

	public Pose2d getPose() {
		return m_odometry.getPoseMeters();
	}

	public void tankDriveVolts(double leftVolts, double rightVolts) {
		SmartDashboard.putNumber("Right Volts - Left Volts", rightVolts - leftVolts);
		SmartDashboard.putNumber("Right Volts", rightVolts);
		SmartDashboard.putNumber("Left Volts",leftVolts);
		talonLeftLeader.setVoltage(leftVolts);
		talonRightLeader.setVoltage(rightVolts);
		robotDrive.feed();
	}

	public double getHeading() {
		return ahrs.getRotation2d().getDegrees();
	}

	public double getAngle() {
        return Math.IEEEremainder(ahrs.getAngle(), 360);
    }

	public void resetOdometry(Pose2d pose) {
		resetEncoders();
		m_odometry.resetPosition(
			ahrs.getRotation2d(), nativeUnitsToDistanceMeters(getLeftEncoder()), nativeUnitsToDistanceMeters(getRightEncoder()), pose);
	}	

	private double getLeftEncoder() {
		return talonLeftLeader.getSelectedSensorPosition() - leftEncoderOffset;
	}

	private double getRightEncoder() {
		return talonRightLeader.getSelectedSensorPosition() - rightEncoderOffset;
	}

	public void resetEncoders() {
		leftEncoderOffset += talonLeftLeader.getSelectedSensorPosition();
		rightEncoderOffset += talonRightLeader.getSelectedSensorPosition();
	}

	public double getPitch() {
		return ahrs.getRoll();
	}

	public void setBraking(boolean braking) {
		if(braking) {
			talonLeftLeader.setNeutralMode(NeutralMode.Brake);
			talonLeftFollowerOne.setNeutralMode(NeutralMode.Brake);
			talonLeftFollowerTwo.setNeutralMode(NeutralMode.Brake);
			talonRightLeader.setNeutralMode(NeutralMode.Brake);
			talonRightFollowerOne.setNeutralMode(NeutralMode.Brake);
			talonRightFollowerTwo.setNeutralMode(NeutralMode.Brake);
		} else {
			talonLeftLeader.setNeutralMode(NeutralMode.Coast);
			talonLeftFollowerOne.setNeutralMode(NeutralMode.Coast);
			talonLeftFollowerTwo.setNeutralMode(NeutralMode.Coast);
			talonRightLeader.setNeutralMode(NeutralMode.Coast);
			talonRightFollowerOne.setNeutralMode(NeutralMode.Coast);
			talonRightFollowerTwo.setNeutralMode(NeutralMode.Coast);
		}
	}

	public void zeroGyroAngle() {
        ahrs.zeroYaw();
    }

	public void calibrate() {
		ahrs.calibrate();
	}

	public Command getTimedDrive(long timeMS, double power) {
        final class Timer {
            long setPoint;
            public Timer() {}
            public boolean isFinished() { return System.currentTimeMillis() >= setPoint; }
        };
        
        Timer timer = new Timer();
        return run(() -> arcadeDrive(power, 0))
			.beforeStarting(Commands.runOnce(() -> {timer.setPoint = System.currentTimeMillis() + timeMS;}))
            .until(timer::isFinished).andThen(runOnce(()-> arcadeDrive(0, 0)));
    }

    public Command getBalanceCommand(double minVoltage, double maxVoltage, double delayThresholdDegPerS, double degreeThreshold) {
		final double maxPitch = 20.0;
		MathUtil.SpeedGetter speedGetter = new MathUtil.SpeedGetter(() -> { return getPitch(); });

		Supplier<Double> fwd = () -> {
			return speedGetter.get() > delayThresholdDegPerS ? 0 :
				MathUtil.scaleMagnitude(getPitch(), 0.0, maxPitch, minVoltage, maxVoltage, 1.5);
		};

		return run(() -> arcadeDrive(fwd.get(), 0)).until(() -> {
			if(Math.abs(getPitch()) < degreeThreshold) {
				counter++;
			} else {
				counter = 0;
			}
			return counter > 50;
		}).andThen(Commands.runOnce(() -> {System.out.println("Stopped");}))
		.andThen(run(() -> {arcadeDrive(0, 0);}));
	}
}