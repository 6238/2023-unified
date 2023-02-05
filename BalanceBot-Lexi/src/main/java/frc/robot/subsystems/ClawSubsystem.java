package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawSubsystem extends SubsystemBase{
    private final Solenoid solenoid;

    public ClawSubsystem() {
        solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
    }

    public void extendSolenoid() {
        System.out.println("Running Extend Solenoid.");
        solenoid.set(true);
    }

    public void retractSolenoid() {
        System.out.println("Running Retract Solenoid.");
        solenoid.set(false);
    }
}
