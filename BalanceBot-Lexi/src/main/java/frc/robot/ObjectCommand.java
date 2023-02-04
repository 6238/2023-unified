package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ObjectCommand extends CommandBase {
    private final double positionTolerance = 1;
    private final double angleTolerance = 5;

    private final double minVoltage = 0.35;
    private final double maxVoltage = 0.6;
    // 0 < minVoltage < maxVoltage < 1.0

    private final DriveSubsystem driveSubsystem; 
    private final PhotonCamera camera;

    private double distance;
    private double yawDelta;

    private double objectHeight;

    /*
     * Creates a new Object Command using the camera to detect the input object
     * @param driveSubsystem the driveSubsystem
     * @param camera the camera
     * @param object 0 for cone, 1 for cube
     */
    public ObjectCommand(DriveSubsystem driveSubsystem, PhotonCamera camera, int object) {
        this.driveSubsystem = driveSubsystem;
        this.camera = camera;
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
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
         // Camera stuff
        var result = camera.getLatestResult();

        // get distance to cone
        if(result.hasTargets()) {
            var target = result.getBestTarget();

            distance = PhotonUtils.calculateDistanceToTargetMeters(
                Constants.cameraHeight, // height off ground
                objectHeight, // height off ground
                Constants.cameraPitch, // pitch relative to ground
                Units.degreesToRadians(target.getPitch())) - 1; // to add a buffer for claw
            
            yawDelta = target.getYaw();
            //double angleDelta = Math.asin(1.11*pixelDelta/Constants.CameraResolutionWidth);
            if(distance >= 5) distance = 5;
            double fwd;
            fwd = distance/5 * (maxVoltage - minVoltage) + minVoltage;

            driveSubsystem.arcadeDrive(fwd, yawDelta);
        }
    }

    @Override
    public boolean isFinished() {
        return (distance <= positionTolerance && Math.abs(yawDelta) < angleTolerance);
    }

    @Override
    public void end(boolean interrupted) {
        camera.setDriverMode(true);
    }
}
