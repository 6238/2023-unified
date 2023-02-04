package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawSubsystem extends SubsystemBase{
    private final Solenoid solenoid;

    public ClawSubsystem() {
        solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
    }
}
