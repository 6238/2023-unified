package frc.robot.commands;

import java.util.LinkedList;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class TrajectoryCommand extends RamseteCommand{
    protected DriveSubsystem driveSubsystem;
    public TrajectoryCommand(DriveSubsystem m_robotDrive, List<Translation2d> internalPoints, Pose2d last) {
        super(
           generateTrajectory(internalPoints, last, m_robotDrive),
           m_robotDrive::getPose,
           new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
           new SimpleMotorFeedforward(
               Constants.ksVolts,
               Constants.kvVoltSecondsPerMeter,
               Constants.kaVoltSecondsSquaredPerMeter),
           Constants.kDriveKinematics,
           m_robotDrive::getWheelSpeeds,
           new PIDController(Constants.kPDriveVel, 0, 0),
           new PIDController(Constants.kPDriveVel, 0, 0),
           // RamseteCommand passes volts to the callback
           m_robotDrive::tankDriveVolts,
           m_robotDrive);
        driveSubsystem = m_robotDrive;
    }

    private static Trajectory generateTrajectory(List<Translation2d> internalPoints, Pose2d last, DriveSubsystem m_robotDrive) {
        var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                Constants.ksVolts,
                Constants.kvVoltSecondsPerMeter,
                Constants.kaVoltSecondsSquaredPerMeter),
            Constants.kDriveKinematics,
            10);

        TrajectoryConfig config =
        new TrajectoryConfig(
            Constants.kMaxSpeedMetersPerSecond,
            Constants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

        Trajectory trajectory =
        TrajectoryGenerator.generateTrajectory(
           // Start at the origin facing the +X direction
           new Pose2d(0, 0, new Rotation2d(0)),
           internalPoints,
           last,
           // Pass config
           config);
        
        m_robotDrive.resetPose(trajectory.getInitialPose());

        return trajectory;
    }
}
