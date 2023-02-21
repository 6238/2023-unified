package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCommand extends CommandBase {

    private final ArmSubsystem armSubsystem; 

    // Multiplier for direction
    private int direction;

    /*
     * Creates a new Object Command using the camera to detect the input object
     * @param driveSubsystem the driveSubsystem
     * @param camera the camera
     * @param object 0 for cone, 1 for cube
     */
    public ArmCommand(ArmSubsystem armSubsystem, boolean up) {
        this.armSubsystem = armSubsystem;
        if (up) {
            direction = 1;
        } else {
            direction = -1;
        }
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        armSubsystem.raiseArm(0.40 * direction);
    }

    @Override
    public void end(boolean interrupt) {
        armSubsystem.reset();
    }
}
