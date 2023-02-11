package frc.robot.commands;

import java.util.LinkedList;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToObjectCommand extends TrajectoryCommand {
    /*
     * Creates a new Object Command using the camera to detect the input object
     * @param driveSubsystem the driveSubsystem
     * @param camera the camera
     * @param object 0 for cone, 1 for cube
     */
    public DriveToObjectCommand(DriveSubsystem driveSubsystem, PhotonCamera camera, int object) {
        super(driveSubsystem, getObjectPosition(object, camera),
            getObjectRotation(camera));
        System.out.println("Rotation Value: " + getObjectRotation(camera));
        LinkedList<Pair<Double,Double>> position = getObjectPosition(object, camera);
        System.out.println("X Position Value: " + position.getFirst());
        System.out.println("Y Position Value: " + position.getLast());
    }

    private static double getObjectRotation(PhotonCamera camera) {
        var result = camera.getLatestResult();
        while (!result.hasTargets()) {
            result = camera.getLatestResult();
        }
        return result.getBestTarget().getYaw();
    }

    private static LinkedList<Pair<Double, Double>> getObjectPosition(int object, PhotonCamera camera) {
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
        while (!result.hasTargets()) {
            result = camera.getLatestResult();
        }
        
        var target = result.getBestTarget();
        double distance = PhotonUtils.calculateDistanceToTargetMeters(
            Constants.cameraHeight, // height off ground
            objectHeight, // height off ground
            Constants.cameraPitch, // pitch relative to ground
            Units.degreesToRadians(target.getPitch())); // to add a buffer for claw
        double yawDelta = target.getYaw();
        
        LinkedList<Pair<Double, Double>> coordinateList = new LinkedList<Pair<Double, Double>>();
        coordinateList.add(new Pair<Double, Double>(distance,
            distance * Math.sin(Math.PI * yawDelta / 180)));

        return coordinateList;
    }
}
