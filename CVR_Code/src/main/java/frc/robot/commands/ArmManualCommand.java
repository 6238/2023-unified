package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmManualCommand extends CommandBase {
    ArmSubsystem armSubsystem;
    Joystick joystick;

    public ArmManualCommand(ArmSubsystem armSubsystem, Joystick joystick) {
        this.armSubsystem = armSubsystem;
        this.joystick = joystick;

        addRequirements(armSubsystem);
    }

    @Override
    public void execute() {
        boolean raise = joystick.getRawButton(Constants.raiseArmBttn);
        boolean lower = joystick.getRawButton(Constants.lowerArmBttn);
        boolean extend = joystick.getRawButton(Constants.extendArmBttn);
        boolean retract = joystick.getRawButton(Constants.retractArmBttn);

        double raiseRate = 0;
        double extendRate = 0;
        
        if(raise && !lower) {
            raiseRate = 0.25; // Divide by 3 for testing
        } else if(!raise && lower) {
            raiseRate = -0.25; // Divide by 3 for testing
        } else {
            raiseRate = 0;
        }

        if(extend && !retract) {
            extendRate = 0.25; // Divide by 3 for testing
        } else if(!extend && retract) {
            extendRate = -0.3; // Divide by 3 for testing
        } else {
            extendRate = 0;
        }

        if (raise || (lower && armSubsystem.canPulleyMoveDown())) {
            armSubsystem.deactivatePulleySetpoint();
        }
        if (extend || retract) {
            armSubsystem.deactivateTelescopeSetpoint();
        }
        
        armSubsystem.raiseArm(raiseRate);
        armSubsystem.extendTelescope(extendRate);
    }

    @Override
    public void end(boolean interrupt) {
        armSubsystem.resetPulley();
        armSubsystem.resetTelescope();
        armSubsystem.activateSetpointModePulley(armSubsystem.getPulleyPosition());
        armSubsystem.activateSetpointModeTelescope(armSubsystem.getTelescopePosition());
    }
}
