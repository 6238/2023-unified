// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

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
  private final Timer timer = new Timer();
  private final Joystick stick = new Joystick(0);

  private final double kCountsPerRev = 2048;
  private final double kGearRatio = 20;
  private final double kWheelRadiusInches = 3;

  private double speed;
  private double rotation;
  private int PID_ID = 0;
  private double timeLimit = 1;

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
  }

  @Override
  public void teleopPeriodic() {
    // Drive with arcade drive.
    // That means that the Y axis drives forward
    // and backward, and the X turns left and right.
    double speed = stick.getY();
    if (speed > .05){
        speed = speed * .65 + .35;
    } else if (speed < -.05) {
        speed = speed * .65 - .35;
    } else {
        speed = 0;
    }

    robotDrive.arcadeDrive(-speed, stick.getX());
    SmartDashboard.putNumber("DriveSpeed", -speed);
    SmartDashboard.putNumber("DriveRotation", rotation);

    SmartDashboard.putNumber("leftMotorsEncoderVelocity", talonLeftLeader.getSelectedSensorVelocity(PID_ID) * 0.1);
    SmartDashboard.putNumber("rightMotorsEncoderVelocity", talonRightLeader.getSelectedSensorVelocity(PID_ID) * 0.1);

    SmartDashboard.putNumber("distanceDriven", getPosition());
  }

  @Override 
  public void autonomousInit() {
    timer.reset();
    timer.start();
  }

  @Override
  public void autonomousPeriodic() {
    double time = timer.get();

    if(time < timeLimit/2) {
      speed = 1;
    } else if(time < timeLimit) {
      speed = -1;
    } else {
      speed = 0;
      System.out.println("Done!");
    }
    rotation = 0;

    robotDrive.arcadeDrive(-speed, -rotation);
    SmartDashboard.putNumber("DriveSpeed", -speed);
    SmartDashboard.putNumber("DriveRotation", rotation);

    SmartDashboard.putNumber("leftMotorsEncoderVelocity", talonLeftLeader.getSelectedSensorVelocity(PID_ID) * 0.1);
    SmartDashboard.putNumber("rightMotorsEncoderVelocity", talonRightLeader.getSelectedSensorVelocity(PID_ID) * 0.1);

    SmartDashboard.putNumber("distanceDriven", getPosition());
  }

  public double getPosition() {
    double position = 
    nativeUnitsToDistanceMeters(
        talonLeftLeader.getSelectedSensorPosition() / 2
        + talonRightLeader.getSelectedSensorPosition() / 2);
    return position;
  }

  private double nativeUnitsToDistanceMeters(double sensorCounts){
    double motorRotations = sensorCounts / kCountsPerRev;
    double wheelRotations = motorRotations / kGearRatio;
    double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches));

    return positionMeters * 2.5106;
  }
}