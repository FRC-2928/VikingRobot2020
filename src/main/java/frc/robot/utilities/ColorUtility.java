package frc.robot.utilities;

public class ColorUtility {
    public enum ControlPanelColor {
        UNKNOWN, RED, GREEN, BLUE, YELLOW
    }

    public ControlPanelColor getClosestColor(double r, double g, double b) {
        return ControlPanelColor.UNKNOWN;
    }
}
