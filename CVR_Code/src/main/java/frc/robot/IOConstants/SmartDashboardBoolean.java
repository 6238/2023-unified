package frc.robot.IOConstants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartDashboardBoolean {
    private final String name;
    public SmartDashboardBoolean(String name, boolean defaultValue) {
        this.name = name;
        boolean uiValue = SmartDashboard.getBoolean(this.name, defaultValue);
        SmartDashboard.putBoolean(this.name, uiValue);
    }

    public boolean get() {
        return SmartDashboard.getBoolean(name, false);
    }
}