// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with
 * arcade steering.
 */
public class Robot extends TimedRobot {
  private final WPI_TalonFX talonLeftLeader = new WPI_TalonFX(Constants.LEFT_LEADER_ID);
  private final WPI_TalonFX talonLeftFollowerOne = new WPI_TalonFX(Constants.LEFT_FOLLOWER_ID_ONE);
  private final WPI_TalonFX talonLeftFollowerTwo = new WPI_TalonFX(Constants.LEFT_FOLLOWER_ID_TWO);
  private final WPI_TalonFX talonRightLeader = new WPI_TalonFX(Constants.RIGHT_LEADER_ID);
  private final WPI_TalonFX talonRightFollowerOne = new WPI_TalonFX(Constants.RIGHT_FOLLOWER_ID_ONE);
  private final WPI_TalonFX talonRightFollowerTwo = new WPI_TalonFX(Constants.RIGHT_FOLLOWER_ID_TWO);
  private final DifferentialDrive robotDrive = new DifferentialDrive(talonLeftLeader, talonRightLeader);
  private final Joystick stick = new Joystick(0);

  private double speed;
  private double rotation;

  private long startTime;
  private long timeLimit = 1000;
  public void setDrive(double speed, double rotation) {
    this.speed = speed;
    this.rotation = rotation;
  }

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    talonLeftFollowerOne.follow(talonLeftLeader);
    talonLeftFollowerTwo.follow(talonLeftLeader);
    talonRightFollowerOne.follow(talonRightLeader);
    talonRightFollowerTwo.follow(talonRightLeader);
    talonLeftLeader.setInverted(true);
    talonLeftFollowerOne.setInverted(true);
    talonLeftFollowerTwo.setInverted(true);

    startTime = System.currentTimeMillis();
    // setDrive(1,0);
  }

  @Override
  public void teleopPeriodic() {
    // Drive with arcade drive.
    // That means that the Y axis drives forward
    // and backward, and the X turns left and right.
    robotDrive.arcadeDrive(-stick.getY(), -stick.getX());
  }

  // @Override
  // public void periodic() {
  //   long timeElapsed = System.currentTimeMillis() - startTime;

  //   robotDrive.arcadeDrive(-speed, -rotation);
  //   SmartDashboard.putNumber("DriveSpeed", -speed);
  //   SmartDashboard.putNumber("DriveRotation", rotation);

  //   SmartDashboard.putNumber("leftMotorsEncoderVelocity", talonLeftLeader.getSelectedSensorVelocity(PID_ID) * 0.1);
  //   SmartDashboard.putNumber("rightMotorsEncoderVelocity", talonRightLeader.getSelectedSensorVelocity(PID_ID) * 0.1);

  //   SmartDashboard.putNumber("distanceDriven", getPosition());
  //   if(timeElapsed > timeLimit/2) {
  //     speed = -1
  //   } else {
  //     speed = 0;
  //   }
  //   rotation = 0;
  // }

  // @Override
  // public boolean isFinished() {
  //   long timeElapsed = System.currentTimeMillis() - startTime;
  //   return timeElapsed > timeLimit;
  // }

  // @Override
  // public void end(boolean interrupted) {
  //     drive.setDrive(0,0);
  //     System.out.println("Done!");
  // }
}