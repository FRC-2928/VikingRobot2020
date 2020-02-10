package frc.org.ballardrobotics.types;

import java.util.function.Consumer;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class PIDValues {
    private double m_p, m_i, m_d, m_f, m_iZone;

    public PIDValues() {
        this(0, 0, 0);
    }

    public PIDValues(double p, double i, double d) {
        this(p, i, d, 0);
    }

    public PIDValues(double p, double i, double d, double f) {
        this(p, i, d, f, 0);
    }

    public PIDValues(double p, double i, double d, double f, double iZone) {
        m_p = p;
        m_i = i;
        m_d = d;
        m_f = f;
        m_iZone = iZone;
    }

    public PIDValues(PIDValues other) {
        this(other.getP(), other.getI(), other.getD(), other.getF(), other.getIZone());
    }

    public static void displayOnShuffleboard(final PIDValues values, String name, final Consumer<PIDValues> onValueChanged) {
        var copy = new PIDValues(values);

        int flags = EntryListenerFlags.kNew | EntryListenerFlags.kUpdate;
        var layout = Shuffleboard.getTab("PIDValues").getLayout(name, BuiltInLayouts.kList);

        layout.add("P", copy.getP()).withWidget(BuiltInWidgets.kTextView).getEntry().addListener((notif) -> {
            copy.setP(notif.value.getDouble());
            onValueChanged.accept(copy);
        }, flags);

        layout.add("I", copy.getI()).withWidget(BuiltInWidgets.kTextView).getEntry().addListener((notif) -> {
            copy.setI(notif.value.getDouble());
            onValueChanged.accept(copy);
        }, flags);

        layout.add("D", copy.getD()).withWidget(BuiltInWidgets.kTextView).getEntry().addListener((notif) -> {
            copy.setD(notif.value.getDouble());
            onValueChanged.accept(copy);
        }, flags);

        layout.add("F", copy.getF()).withWidget(BuiltInWidgets.kTextView).getEntry().addListener((notif) -> {
            copy.setF(notif.value.getDouble());
            onValueChanged.accept(copy);
        }, flags);

        layout.add("iZone", copy.getIZone()).withWidget(BuiltInWidgets.kTextView).getEntry().addListener((notif) -> {
            copy.setIZone(notif.value.getDouble());
            onValueChanged.accept(copy);
        }, flags);

        onValueChanged.accept(copy);
    }

    private synchronized void setP(double p) {
        m_p = p;
    }

    private synchronized void setI(double i) {
        m_i = i;
    }

    private synchronized void setD(double d) {
        m_d = d;
    }

    private synchronized void setF(double f) {
        m_f = f;
    }

    private synchronized void setIZone(double iZone) {
        m_iZone = iZone;
    }

    public synchronized double getP() {
        return m_p;
    }

    public synchronized double getI() {
        return m_i;
    }

    public synchronized double getD() {
        return m_d;
    }

    public synchronized double getF() {
        return m_f;
    }

    public synchronized double getIZone() {
        return m_iZone;
    }

    public PIDValues withP(double p) {
        return new PIDValues(p, m_i, m_d, m_f, m_iZone);
    }

    public PIDValues withI(double i) {
        return new PIDValues(m_p, i, m_d, m_f, m_iZone);
    }

    public PIDValues withD(double d) {
        return new PIDValues(m_p, m_i, d, m_f, m_iZone);
    }

    public PIDValues withF(double f) {
        return new PIDValues(m_p, m_i, m_d, f, m_iZone);
    }

    public PIDValues withIZone(double iZone) {
        return new PIDValues(m_p, m_i, m_d, m_f, iZone);
    }
}
