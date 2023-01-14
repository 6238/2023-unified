package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public class Constants {
    public final static int LEFT_LEADER_ID = 30;
    public final static int LEFT_FOLLOWER_ID_ONE = 32;
    public final static int LEFT_FOLLOWER_ID_TWO = 34;
    public final static int RIGHT_LEADER_ID = 31;
    public final static int RIGHT_FOLLOWER_ID_ONE = 33;
    public final static int RIGHT_FOLLOWER_ID_TWO = 35;
    
    // Track width = horizontal distance between wheels
    public static final double kTrackwidthMeters = 0.65; // Should be the same if we use the same frame and drivetrain
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

    // Max speed and acceleration
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;

    // Feedforward gains (Robot specific, calibrate according to: https://docs.wpilib.org/en/stable/docs/software/pathplanning/trajectory-tutorial/characterizing-drive.html)
    public final static double ksVolts = 0.15547;
    public final static double kvVoltSecondsPerMeter = 1.9825;
    public final static double kaVoltSecondsSquaredPerMeter = 0.21002;

    // Feedback gains (Robot specific, calibrate as above)
    public final static double kPDriveVel = 1.2122;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
}
