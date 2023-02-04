package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ConeCommand extends CommandBase {
    private final double positionTolerance = 1;
    private final double angleTolerance = 5;

    private final double minVoltage = 0.35;
    private final double maxVoltage = 0.6;
    // 0 < minVoltage < maxVoltage < 1.0

    private final DriveSubsystem driveSubsystem; 
    private final PhotonCamera camera;

    double distance;
    double yawDelta;

    public ConeCommand(DriveSubsystem driveSubsystem, PhotonCamera camera) {
        this.driveSubsystem = driveSubsystem;
        this.camera = camera;
        camera.setDriverMode(false);
        camera.setPipelineIndex(0);
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
                Constants.coneHeight, // height off ground
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
