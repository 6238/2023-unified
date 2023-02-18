// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with
 * arcade steering.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    SmartDashboard.putBoolean("Brakes", true);
    m_robotContainer.setBraking(true);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    boolean braking = SmartDashboard.getBoolean("Brakes", true);
    if(braking != m_robotContainer.isBraking()) {
      m_robotContainer.setBraking(braking);
      SmartDashboard.putBoolean("Brakes", braking);
    }
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    SmartDashboard.putNumber("Distance To Travel", SmartDashboard.getNumber("Distance To Travel", 1));
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}



  // @Override
  // public void robotInit() {
  //   // We need to invert one side of the drivetrain so that positive voltages
  //   // result in both sides moving forward. Depending on how your robot's
  //   // gearbox is constructed, you might have to invert the left side instead.
  //   talonLeftFollowerOne.follow(talonLeftLeader);
  //   talonLeftFollowerTwo.follow(talonLeftLeader);
  //   talonRightFollowerOne.follow(talonRightLeader);
  //   talonRightFollowerTwo.follow(talonRightLeader);
  //   talonLeftLeader.setInverted(true);
  //   talonLeftFollowerOne.setInverted(true);
  //   talonLeftFollowerTwo.setInverted(true);
  // }

  // @Override
  // public void teleopPeriodic() {
  //   // Drive with arcade drive.
  //   // That means that the Y axis drives forward
  //   // and backward, and the X turns left and right.
  //   double speed = stick.getY();
  //   if (speed > .05){
  //       speed = speed * .65 + .35;
  //   } else if (speed < -.05) {
  //       speed = speed * .65 - .35;
  //   } else {
  //       speed = 0;
  //   }

  //   robotDrive.arcadeDrive(-speed, stick.getX());
  //   SmartDashboard.putNumber("DriveSpeed", -speed);
  //   SmartDashboard.putNumber("DriveRotation", rotation);

  //   SmartDashboard.putNumber("leftMotorsEncoderVelocity", talonLeftLeader.getSelectedSensorVelocity(PID_ID) * 0.1);
  //   SmartDashboard.putNumber("rightMotorsEncoderVelocity", talonRightLeader.getSelectedSensorVelocity(PID_ID) * 0.1);

  //   SmartDashboard.putNumber("distanceDriven", getPosition());
  // }

  // @Override 
  // public void autonomousInit() {
  //   timer.reset();
  //   timer.start();
  // }


  // //a little startup time needed (0.25?)
  // //needs a little time to switch directions if going fast
  // @Override
  // public void autonomousPeriodic() {
  //   double time = timer.get();

  //   if(time < timeLimit/2) {
  //     speed = 0.4;
  //   } else if(time < timeLimit/2+0.1) {
  //     speed = 0;
  //   }else if(time < timeLimit) {
  //     speed = -0.4;
  //   } else {
  //     speed = 0;
  //   }
  //   rotation = 0;

  //   robotDrive.arcadeDrive(speed, rotation);
  //   SmartDashboard.putNumber("DriveSpeed", -speed);
  //   SmartDashboard.putNumber("DriveRotation", rotation);

  //   SmartDashboard.putNumber("leftMotorsEncoderVelocity", talonLeftLeader.getSelectedSensorVelocity(PID_ID) * 0.1);
  //   SmartDashboard.putNumber("rightMotorsEncoderVelocity", talonRightLeader.getSelectedSensorVelocity(PID_ID) * 0.1);

  //   SmartDashboard.putNumber("distanceDriven", getPosition());
  // }