// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import org.photonvision.PhotonCamera;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SmartDashboardParam;

public class DriveSubsystem extends SubsystemBase {
	private final SmartDashboardParam currentLimit = new SmartDashboardParam("currentLimiter", 16);

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

	private final int PID_ID;

	private final DifferentialDrivePoseEstimator poseEstimator;

	private final PhotonCamera camera;

	private boolean TargetModeOn = false;
	private Pose3d targetPose;

	private final Transform3d cameraToRobot = new Transform3d(
		new Pose3d(Constants.cameraHeight, 0.0, Constants.kTrackwidthMeters / 2,
			new Rotation3d(0.0, Constants.cameraPitch, 0.0)),
		new Pose3d());

	public DriveSubsystem(PhotonCamera camera) {
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
		zeroGyroAngle();

		SmartDashboard.putBoolean("Brakes", true);
		setBraking(true);

		this.camera = camera;
		poseEstimator = new DifferentialDrivePoseEstimator(Constants.kDriveKinematics,
			new Rotation2d(),
			0.0,
			0.0,
			new Pose2d());
	}

	public void toggleTargetMode(Pose3d target, int object) {
		resetPose();
		TargetModeOn ^= true;
		targetPose = target;
        camera.setDriverMode(false);
        camera.setPipelineIndex(object);
	}

	@Override
	public void periodic() {
		poseEstimator.update(ahrs.getRotation2d(), nativeUnitsToDistanceMeters(getLeftEncoder()), nativeUnitsToDistanceMeters(getRightEncoder()));
		if (TargetModeOn) {
			var result = camera.getLatestResult();
			if (result == null) {
				return;
			}

			double imageCaptureTime = result.getTimestampSeconds();
            Transform3d camToTarget = result.getBestTarget().getBestCameraToTarget();
            Pose3d camPose = targetPose.transformBy(camToTarget.inverse());
            poseEstimator.addVisionMeasurement(
                    camPose.transformBy(cameraToRobot).toPose2d(), imageCaptureTime);
		}
		SmartDashboard.putNumber("X Position", Math.floor(getPose().getX()*1000)/1000);
        SmartDashboard.putNumber("Y Position", Math.floor(getPose().getY()*1000)/1000);
        SmartDashboard.putNumber("Angle Position", Math.floor(getPose().getRotation().getDegrees()*1000)/1000);
		SmartDashboard.putNumber("X Position Graph", Math.floor(getPose().getX()*1000)/1000);
		SmartDashboard.putNumber("Y Position Graph", Math.floor(getPose().getY()*1000)/1000);
        SmartDashboard.putNumber("Angle Position Graph", Math.floor(getPose().getRotation().getDegrees()*1000)/1000);
		SmartDashboard.putNumber("Pitch Angle", ahrs.getPitch());

		boolean braking = SmartDashboard.getBoolean("Brakes", true);
		setBraking(braking);
	}

	public void arcadeDrive(double fwd, double rot) {
		robotDrive.arcadeDrive(fwd, -rot);
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
		return poseEstimator.getEstimatedPosition();
	}

	public void tankDriveVolts(double leftVolts, double rightVolts) {
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

	public void resetPose() {
		resetEncoders();
		poseEstimator.resetPosition(
			ahrs.getRotation2d(),
			nativeUnitsToDistanceMeters(getLeftEncoder()),
			nativeUnitsToDistanceMeters(getRightEncoder()),
			new Pose2d());
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
		System.out.println("Pitch : " + ahrs.getPitch());
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

	public void zeroGyroAngle() {
        ahrs.zeroYaw();
    }
}