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
    private int armDirection;
    private int telescopeDirection;

    public ArmCommand(ArmSubsystem armSubsystem, boolean isRaisingArm, boolean isExtendingTelescope) {
        this.armSubsystem = armSubsystem;
        armDirection = isRaisingArm ? 1 : -1;
        telescopeDirection = isExtendingTelescope ? 1 : -1;
    }

    @Override
    public void execute() {
        armSubsystem.raiseArm(0.40 * armDirection);
        telescopeDirection.extendTelescope(0.40 * telescopeDirection);
    }

    @Override
    public void end(boolean interrupt) {
        armSubsystem.reset();
    }
}
