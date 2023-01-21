// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SmartDashboardParam;

public class DriveSubsystem extends SubsystemBase {
	private final SmartDashboardParam currentLimit = new SmartDashboardParam("currentLimiter", 16); // 0.18

	private final double kCountsPerRev = 2048;
  	private final double kGearRatio = 20;
  	private final double kWheelRadiusInches = 2.75;

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

	private final int PID_ID;
    boolean isBraking;

	public DriveSubsystem() {
		PID_ID = 0;
        isBraking = true;
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

        talonLeftLeader.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, PID_ID, 0);
        talonRightLeader.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, PID_ID, 0);

        talonLeftLeader.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, currentLimit.get(), 0, 0));
        talonLeftFollowerOne.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, currentLimit.get(), 0, 0));
        talonLeftFollowerTwo.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, currentLimit.get(), 0, 0));
        talonRightLeader.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, currentLimit.get(), 0, 0));
        talonRightFollowerOne.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, currentLimit.get(), 0, 0));
        talonRightFollowerTwo.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, currentLimit.get(), 0, 0));

		resetEncoders();
		zeroGyroAngle();

		m_odometry = new DifferentialDriveOdometry(ahrs.getRotation2d(), nativeUnitsToDistanceMeters(talonLeftLeader.getSelectedSensorPosition()), nativeUnitsToDistanceMeters(talonRightLeader.getSelectedSensorPosition()));
	}

	@Override
	public void periodic() {
		m_odometry.update(ahrs.getRotation2d(), nativeUnitsToDistanceMeters(getLeftEncoder()), nativeUnitsToDistanceMeters(getRightEncoder()));
		SmartDashboard.putNumber("X Position", getPose().getX());
        SmartDashboard.putNumber("Y Position", getPose().getY());
        SmartDashboard.putNumber("Angle Position", getPose().getRotation().getDegrees());
		SmartDashboard.putNumber("X Position Graph", getPose().getX());
		SmartDashboard.putNumber("Y Position Graph", getPose().getY());
        SmartDashboard.putNumber("Angle Position Graph", getPose().getRotation().getDegrees());
	}

	public void arcadeDrive(double fwd, double rot) {
		robotDrive.arcadeDrive(fwd, -rot);
	}

	private double nativeUnitsToDistanceMeters(double sensorCounts){
        double motorRotations = sensorCounts / kCountsPerRev;
        double wheelRotations = motorRotations / kGearRatio;
        double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches));

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
		return ahrs.getPitch();
	}

	public void zeroGyroAngle() {
        ahrs.zeroYaw();
    }
}