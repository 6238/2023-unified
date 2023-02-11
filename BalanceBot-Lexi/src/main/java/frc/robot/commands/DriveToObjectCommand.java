package frc.robot.commands;

import java.util.LinkedList;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToObjectCommand extends CommandBase {
    /*
     * Creates a new Object Command using the camera to detect the input object
     * @param driveSubsystem the driveSubsystem
     * @param camera the camera
     * @param object 0 for cone, 1 for cube
     */

    private TrajectoryCommand command;
    private DriveSubsystem driveSubsystem;
    private PhotonCamera camera;
    private int object;

    public DriveToObjectCommand(DriveSubsystem driveSubsystem, PhotonCamera camera, int object) {
        this.driveSubsystem = driveSubsystem;
        this.camera = camera;
        this.object = object;
    }

    @Override
    public void initialize() {
        command = new TrajectoryCommand(driveSubsystem,
        new LinkedList<Pair<Double, Double>>(getObjectPosition(object, camera)),
        0);
        command.schedule();
    }

    @Override
    public void end(boolean interrupted) {
        command.cancel();
    }

    private static LinkedList<Pair<Double, Double>> getObjectPosition(int object, PhotonCamera camera) {
        System.out.println("Inside getObjectPosition");
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
            System.out.println("Stuck in loop.");
            result = camera.getLatestResult();
        }
        System.out.println("End Loop.");
        
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
        for (int i = 0; i < 1000; i++) {
            System.out.println("X Position Target: " + coordinateList.get(0).getFirst());
        }
        for (int j = 0; j < 1000; j++) {
            System.out.println("Y Position Target: " + coordinateList.get(0).getSecond());
        }

        return coordinateList;
    }
}
