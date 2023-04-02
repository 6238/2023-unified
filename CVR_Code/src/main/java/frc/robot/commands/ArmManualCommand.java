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

        int raiseRate = 0;
        int extendRate = 0;
        
        if(raise && !lower) {
            raiseRate = 1;
        } else if(!raise && lower) {
            raiseRate = -1;
        } else {
            raiseRate = 0;
        }

        if(extend && !retract) {
            extendRate = 1;
        } else if(!extend && retract) {
            extendRate = -1;
        } else {
            extendRate = 0;
        }

        armSubsystem.deactivateSetpointMode();
        armSubsystem.raiseArm(raiseRate);
        armSubsystem.extendTelescope(extendRate);
    }

    @Override
    public void end(boolean interrupt) {
        armSubsystem.resetPulley();
        armSubsystem.resetTelescope();
        armSubsystem.activateSetpointMode(armSubsystem.getPulleyPosition(),
            armSubsystem.getTelescopePosition());
    }
}
