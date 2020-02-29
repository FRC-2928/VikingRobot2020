package frc.robot.utilities;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.types.ControlPanelColor;

/**
 * ColorMatcher takes RGB input and outputs a color.
 */
public class ColorMatcher {
    private final ColorMatch m_colorMatcher = new ColorMatch();

    private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
    private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
    private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
    private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

    public ColorMatcher() {
        m_colorMatcher.addColorMatch(kBlueTarget);
        m_colorMatcher.addColorMatch(kGreenTarget);
        m_colorMatcher.addColorMatch(kRedTarget);
        m_colorMatcher.addColorMatch(kYellowTarget);
    }

    public ControlPanelColor getMatchedColor(Color detectedColor) {
        ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

        if (match.color == kRedTarget) {
            return ControlPanelColor.RED;
        }
        if (match.color == kGreenTarget) {
            return ControlPanelColor.GREEN;
        }
        if (match.color == kBlueTarget) {
            return ControlPanelColor.BLUE;
        }
        if (match.color == kYellowTarget) {
            return ControlPanelColor.YELLOW;
        }

        return ControlPanelColor.UNKNOWN;
    }
}
