// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
	/** Creates a new ExampleSubsystem. */
	
	private final MotorControllerGroup  left = new MotorControllerGroup(
		new WPI_TalonFX(Constants.LEFT_LEADER_ID), 
		new WPI_TalonFX(Constants.LEFT_FOLLOWER_ONE_ID),
		new WPI_TalonFX(Constants.LEFT_FOLLOWER_TWO_ID)
	);

	private final MotorControllerGroup right = new MotorControllerGroup(
		new WPI_TalonFX(Constants.RIGHT_LEADER_ID),
		new WPI_TalonFX(Constants.RIGHT_FOLLOWER_ONE_ID),
		new WPI_TalonFX(Constants.RIGHT_FOLLOWER_TWO_ID)
	);

	private final DifferentialDrive drive = new DifferentialDrive(left, right);

	public void arcadeDrive(double rot, double fwd) {
		drive.arcadeDrive(rot, fwd);
	}
}
