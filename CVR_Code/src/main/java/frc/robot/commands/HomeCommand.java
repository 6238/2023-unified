package frc.robot.commands;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class HomeCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem m_subsystem;

  public HomeCommand(ArmSubsystem subsystem) {
    m_subsystem = subsystem;
    

    addRequirements(subsystem);
  } 
  @Override
  public void execute(){
    System.out.println("homecommandrunning");
    m_subsystem.raiseArm(1);
    m_subsystem.extendTelescope(-1);
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.raiseArm(0);
    m_subsystem.extendTelescope(0);
  }

}