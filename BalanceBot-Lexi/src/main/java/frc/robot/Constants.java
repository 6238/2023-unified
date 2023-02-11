package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public class Constants {
    public final static int LEFT_LEADER_ID = 31;
    public final static int LEFT_FOLLOWER_ID_ONE = 33;
    public final static int LEFT_FOLLOWER_ID_TWO = 35;
    public final static int RIGHT_LEADER_ID = 30;
    public final static int RIGHT_FOLLOWER_ID_ONE = 32;
    public final static int RIGHT_FOLLOWER_ID_TWO = 34;

    public final static int CameraResolutionWidth = 180;
    public final static int CameraResolutionHeight = 320;
    public final static double cameraHeight = 1.384; // meters
    public final static double coneHeight = 0.165; // meters to center of cone(assuming it is on the floor)
    public final static double cubeHeight = 0.12; // meters to center of cube(assuming it is on the floor)
    public final static double cameraPitch = -0.96; // radians
    
    public final static double kCountsPerRev = 2048;
    public final static double kGearRatio = 20;
    public final static double kWheelRadiusInches = 2.75;
    
    // Track width = horizontal distance between wheels
    public static final double kTrackwidthMeters = 0.65; // Should be the same if we use the same frame and drivetrain
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

    // Max speed and acceleration
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;

    // Feedforward gains (Robot specific, calibrate according to: https://docs.wpilib.org/en/stable/docs/software/pathplanning/trajectory-tutorial/characterizing-drive.html)
    public final static double ksVolts = 0.09973;//0.13131;
    public final static double kvVoltSecondsPerMeter = 2.1491;//2.1269;
    public final static double kaVoltSecondsSquaredPerMeter = 0.16595;//0.37915;

    // Feedback gains (Robot specific, calibrate as above)
    //public final static double kPDriveVel = 1.2122;
     public final static double kPDriveVel = 0.29573;//1.1291;;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    // PID values for stabilization
    public static double kpStabilization = 0;
    public static double kiStabilization = 0;
    public static double kdStabilization = 0;

    public static int BalanceRobotBttn = 2;
    public static int DriveToObjBttn = 10;
    public static int OpenClawBttn = 1;
}
