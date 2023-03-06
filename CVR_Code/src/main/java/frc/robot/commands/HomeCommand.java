package frc.robot.commands;
import frc.robot.subsystems.ArmSubsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class HomeCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem m_subsystem;
  private boolean isInterrupted = false;

  private final long maxTimer = 500;
  private long startTime;

  public HomeCommand(ArmSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  } 

  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
    isInterrupted = false;
  }

  @Override
  public void execute(){
    System.out.println("homecommandrunning");
    m_subsystem.raiseArm(1);
    m_subsystem.extendTelescope(-1);
  }

  @Override
  public boolean isFinished() {
    if (System.currentTimeMillis() - startTime > maxTimer) {
      if (m_subsystem.isPulleyStalled()) {
        System.out.println("Arm Stalled.");
      }
      if (m_subsystem.isTelescopeStalled()) {
        System.out.println("Telescope Stalled.");
      }
    }
    double timePassed = System.currentTimeMillis() - startTime;
    System.out.println("Time Passed: " + timePassed);
    System.out.println("Running Home Command:" + (timePassed > maxTimer && m_subsystem.isPulleyStalled() && m_subsystem.isTelescopeStalled()));
    return isInterrupted || (timePassed > maxTimer && m_subsystem.isPulleyStalled() && m_subsystem.isTelescopeStalled());
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.raiseArm(0);
    m_subsystem.extendTelescope(0);
    isInterrupted = interrupted;
  }
}