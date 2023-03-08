package frc.robot.IOConstants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartDashboardNumeric {
    private final String name;
    public SmartDashboardNumeric (String name, double defaultValue) {
        this.name = name;
        double uiValue = SmartDashboard.getNumber(this.name, defaultValue);
        SmartDashboard.putNumber(this.name, uiValue);
    }

    public double get() {
        return SmartDashboard.getNumber(name, 0);
    }
}