package frc.org.ballardrobotics.types;

import java.util.function.Consumer;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class PIDValues {
    public static final Consumer<PIDValues> kDefaultConsumer = (values) -> {
        System.out.println("p = " + values.getP());
        System.out.println("i = " + values.getI());
        System.out.println("d = " + values.getD());
        System.out.println("f = " + values.getF());
        System.out.println("iZone = " + values.getIZone());
    };

    public static void displayOnShuffleboard(String tab, String name, PIDValues value) {
        displayOnShuffleboard(tab, name, value, (newValue) -> {});
    }

    public static void displayOnShuffleboard(String tab, String name, PIDValues value, final Consumer<PIDValues> onValueChanged) {        
        int flags = EntryListenerFlags.kNew | EntryListenerFlags.kUpdate;
        var layout = Shuffleboard.getTab(tab).getLayout(name);

        layout.add("P", value.getP()).withWidget(BuiltInWidgets.kTextView).getEntry().addListener((notif) -> {
            value.setP(notif.value.getDouble());
            onValueChanged.accept(value);
        }, flags);

        layout.add("I", value.getI()).withWidget(BuiltInWidgets.kTextView).getEntry().addListener((notif) -> {
            value.setI(notif.value.getDouble());
            onValueChanged.accept(value);
        }, flags);

        layout.add("D", value.getD()).withWidget(BuiltInWidgets.kTextView).getEntry().addListener((notif) -> {
            value.setD(notif.value.getDouble());
            onValueChanged.accept(value);
        }, flags);

        layout.add("F", value.getF()).withWidget(BuiltInWidgets.kTextView).getEntry().addListener((notif) -> {
            value.setF(notif.value.getDouble());
            onValueChanged.accept(value);
        }, flags);

        layout.add("iZone", value.getIZone()).withWidget(BuiltInWidgets.kTextView).getEntry().addListener((notif) -> {
            value.setIZone(notif.value.getDouble());
            onValueChanged.accept(value);
        }, flags);
    }

    private double m_p, m_i, m_d, m_f, m_iZone;

    public PIDValues(double p, double i, double d, double f, double iZone) {
        m_p = p;
        m_i = i;
        m_d = d;
        m_f = f;
        m_iZone = iZone;
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

    public synchronized PIDValues withP(double p) {
        return new PIDValues(p, m_i, m_d, m_f, m_iZone);
    }

    public synchronized PIDValues withI(double i) {
        return new PIDValues(m_p, i, m_d, m_f, m_iZone);
    }

    public synchronized PIDValues withD(double d) {
        return new PIDValues(m_p, m_i, d, m_f, m_iZone);
    }

    public synchronized PIDValues withF(double f) {
        return new PIDValues(m_p, m_i, m_d, f, m_iZone);
    }

    public synchronized PIDValues withIZone(double iZone) {
        return new PIDValues(m_p, m_i, m_d, m_f, iZone);
    }
}
