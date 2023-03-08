// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.IOConstants.SmartDashboardNumeric;

public class DriveSubsystem extends SubsystemBase {
	private final SmartDashboardNumeric currentLimit = new SmartDashboardNumeric("currentLimiter", 16); // 0.18

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

	public DriveSubsystem() {
		PID_ID = 0;
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
		ahrs.zeroYaw();

		SmartDashboard.putBoolean("Brakes", true);
		setBraking(true);

		m_odometry = new DifferentialDriveOdometry(ahrs.getRotation2d(), nativeUnitsToDistanceMeters(talonLeftLeader.getSelectedSensorPosition()), nativeUnitsToDistanceMeters(talonRightLeader.getSelectedSensorPosition()));
	}

	@Override
	public void periodic() {
		m_odometry.update(ahrs.getRotation2d(), nativeUnitsToDistanceMeters(getLeftEncoder()), nativeUnitsToDistanceMeters(getRightEncoder()));
		
		// For Testing Purposes
		SmartDashboard.putNumber("X Position", Math.floor(getPose().getX()*1000)/1000);
        SmartDashboard.putNumber("Y Position", Math.floor(getPose().getY()*1000)/1000);
        SmartDashboard.putNumber("Angle Position", Math.floor(getPose().getRotation().getDegrees()*1000)/1000);
		SmartDashboard.putNumber("X Position Graph", Math.floor(getPose().getX()*1000)/1000);
		SmartDashboard.putNumber("Y Position Graph", Math.floor(getPose().getY()*1000)/1000);
        SmartDashboard.putNumber("Angle Position Graph", Math.floor(getPose().getRotation().getDegrees()*1000)/1000);
		SmartDashboard.putNumber("Pitch Angle", ahrs.getPitch());
		// For Testing Purposes

		boolean braking = SmartDashboard.getBoolean("Brakes", true);
		setBraking(braking);
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

	public Pose2d getPose() {
		return m_odometry.getPoseMeters();
	}

	public void tankDriveVolts(double leftVolts, double rightVolts) {
		talonLeftLeader.setVoltage(-leftVolts);
		talonRightLeader.setVoltage(-rightVolts);
		robotDrive.feed();
	}

	public void resetOdometry(Pose2d pose) {
		resetEncoders();
		ahrs.zeroYaw();
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
		leftEncoderOffset += talonLeftLeader.getSelectedSensorPosition(); // J.T. 2023-03-04 not sure that these should be "+=" instead of "="
		rightEncoderOffset += talonRightLeader.getSelectedSensorPosition(); // J.t. 2023-03-04 not sure that these should be "+=" instead of "="
	}

	public double getPitch() {
		return ahrs.getPitch();
	}

	private void setBraking(boolean braking) {
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

	public Pose2d getPoseToObject() {
		Pose2d pose = m_odometry.getPoseMeters();
		return new Pose2d(pose.getX(), pose.getY(), pose.getRotation());
	}
}