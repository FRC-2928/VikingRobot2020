package frc.org.ballardrobotics.types;

import java.util.function.Consumer;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class ProfileValues {
    private double m_maxVelocity, m_maxAcceleration;

    public ProfileValues() {
        this(0.0, 0.0);
    }

    public ProfileValues(double maxVelocity, double maxAcceleration) {
        m_maxVelocity = maxVelocity;
        m_maxAcceleration = maxAcceleration;
    }

    public ProfileValues(ProfileValues other) {
        this(other.getMaxVelocity(), other.getMaxAcceleration());
    }

    public static void displayOnShuffleboard(final ProfileValues values, String name, final Consumer<ProfileValues> onValueChanged) {
        var copy = new ProfileValues(values);

        int flags = EntryListenerFlags.kNew | EntryListenerFlags.kUpdate;
        var layout = Shuffleboard.getTab("ProfileValues").getLayout(name, BuiltInLayouts.kList);

        layout.add("MaxVelocity", copy.getMaxVelocity()).withWidget(BuiltInWidgets.kTextView).getEntry().addListener((notif) -> {
            copy.setMaxVelocity(notif.value.getDouble());
            onValueChanged.accept(copy);
        }, flags);

        layout.add("MaxAcceleration", copy.getMaxAcceleration()).withWidget(BuiltInWidgets.kTextView).getEntry().addListener((notif) -> {
            copy.setMaxAcceleration(notif.value.getDouble());
            onValueChanged.accept(copy);
        }, flags);

        onValueChanged.accept(copy);
    }

    public void setMaxVelocity(double maxVelocity) {
        m_maxVelocity = maxVelocity;
    }

    public void setMaxAcceleration(double maxAcceleration) {
        m_maxAcceleration = maxAcceleration;
    }

    public double getMaxVelocity() {
        return m_maxVelocity;
    }

    public double getMaxAcceleration() {
        return m_maxAcceleration;
    }

    public ProfileValues withMaxVelocity(double maxVelocity) {
        return new ProfileValues(maxVelocity, m_maxAcceleration);
    }

    public ProfileValues withMaxAcceleration(double maxAcceleration) {
        return new ProfileValues(m_maxVelocity, maxAcceleration);
    }
}
