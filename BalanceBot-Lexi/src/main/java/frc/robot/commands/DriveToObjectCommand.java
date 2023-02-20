package frc.robot.commands;

import java.util.LinkedList;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToObjectCommand extends TrajectoryCommand {
    private static int timer = 0;
    
    private Pose2d position;
    
    /*
     * Creates a new Object Command using the camera to detect the input object
     * @param driveSubsystem the driveSubsystem
     * @param camera the camera
     * @param object 0 for cone, 1 for cube
     */
    public DriveToObjectCommand(DriveSubsystem driveSubsystem, PhotonCamera camera, int object) {
        super(driveSubsystem, new LinkedList<Translation2d>(), getObjectPosition(object, camera));
        position = getObjectPosition(object, camera);
        System.out.println("Rotation Value: " + position.getRotation());
        System.out.println("X Position Value: " + position.getX());
        System.out.println("Y Position Value: " + position.getY());
    }

    private static Pose2d getObjectPosition(int object, PhotonCamera camera) {
        double objectHeight = 0;
        switch(object) {
            case 0:
                objectHeight = Constants.coneHeight;
                break;
            case 1:
                objectHeight = Constants.cubeHeight;
                break;
        }

        camera.setDriverMode(false);
        camera.setPipelineIndex(object);

        var result = camera.getLatestResult();
        while (!result.hasTargets() && timer < 50) {
            result = camera.getLatestResult();
            timer++;
        }
        timer = 0;
        
        var target = result.getBestTarget();
        double distance = PhotonUtils.calculateDistanceToTargetMeters(
            Constants.cameraHeight, // height off ground
            objectHeight, // height off ground
            Constants.cameraPitch, // pitch relative to ground
            Units.degreesToRadians(target.getPitch())); // to add a buffer for claw
        double yawDelta = target.getYaw();
        
        return new Pose2d(distance * Math.cos(Math.PI * yawDelta / 180),
            distance * Math.sin(Math.PI * yawDelta / 180),
            new Rotation2d(yawDelta));
    }

    @Override
    public boolean isFinished() {
        return driveSubsystem.getPose().getX() - position.getX() < 0.5
            && driveSubsystem.getPose().getY() - position.getY() < 0.5;
    }
}
