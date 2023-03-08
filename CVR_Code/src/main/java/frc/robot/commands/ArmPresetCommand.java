package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmPresetCommand extends CommandBase {
    private final ArmSubsystem m_subsystem;
    private final double m_pulleyPositionTarget;
    private final double m_telescopePositionTarget;
    
    public ArmPresetCommand(ArmSubsystem subsystem, double pulleyPostionTarget, double telescopePositionTarget) {
        m_subsystem = subsystem;
        m_pulleyPositionTarget = pulleyPostionTarget;
        m_telescopePositionTarget = telescopePositionTarget;
        addRequirements(subsystem);

    }

    @Override
    public void execute(){
        double pulleyPosition = m_subsystem.getPulleyPosition();
        double telescopePosition = m_subsystem.getTelescopePosition();
        double pulleyPositionSpeed = 0;
        double telescopePositionSpeed = 0;
        if (isPulleyPositionAtTarget()){
            pulleyPositionSpeed = 0;
        } else if (m_pulleyPositionTarget < pulleyPosition){
            pulleyPositionSpeed = 1;
        } else if (m_pulleyPositionTarget > pulleyPosition){
            pulleyPositionSpeed = -1;
        }
        if (isTelescopePositionAtTarget()) {
            telescopePosition =0;
        } else if (m_telescopePositionTarget<telescopePosition){
            telescopePositionSpeed = -1;
        } else if (m_telescopePositionTarget>telescopePosition){
            telescopePositionSpeed = 1;
        }
        m_subsystem.raiseArm(pulleyPositionSpeed);
        m_subsystem.extendTelescope(telescopePositionSpeed);
    }

    private boolean isPulleyPositionAtTarget(){
        double pulleyPosition = m_subsystem.getPulleyPosition();
        return Math.abs(m_pulleyPositionTarget-pulleyPosition)< 3;
    }
    private boolean isTelescopePositionAtTarget(){
        double telescopePosition = m_subsystem.getTelescopePosition();
        return Math.abs(m_telescopePositionTarget-telescopePosition)< 3;
    }

    @Override
    public boolean isFinished(){
        return  isPulleyPositionAtTarget() && isTelescopePositionAtTarget();
    }
    @Override
    public void end(boolean interrupted) {
      m_subsystem.raiseArm(0);
      m_subsystem.extendTelescope(0);
    }

}
