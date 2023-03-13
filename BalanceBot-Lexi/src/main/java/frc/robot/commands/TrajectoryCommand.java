package frc.robot.commands;

import java.util.LinkedList;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.Pair;
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
    public TrajectoryCommand(DriveSubsystem m_robotDrive, Pose2d first, List<Pair<Double,Double>> points, Pose2d last) {
        this(m_robotDrive, first, points, last, m_robotDrive::getPose);
    }

    public TrajectoryCommand(DriveSubsystem m_robotDrive, Pose2d first, List<Pair<Double,Double>> points, Pose2d last, Supplier<Pose2d> pose) {
        super(
           generateTrajectory(first, points, last, m_robotDrive),
           pose,
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
    }

    private static Trajectory generateTrajectory(Pose2d first, List<Pair<Double,Double>> points, Pose2d last, DriveSubsystem m_robotDrive) {
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

        List<Translation2d> internalPoints = new LinkedList<Translation2d>();
        for(int i = 0; i < points.size(); i++) {
            Pair<Double,Double> point = points.get(i);
            internalPoints.add(new Translation2d(point.getFirst(), point.getSecond()));
        }

        Trajectory trajectory =
        TrajectoryGenerator.generateTrajectory(
           // Start at the origin facing the +X direction
           first,
           internalPoints,
           last,
           // Pass config
           config);
        
        m_robotDrive.resetOdometry(trajectory.getInitialPose());

        return trajectory;
    }
}
