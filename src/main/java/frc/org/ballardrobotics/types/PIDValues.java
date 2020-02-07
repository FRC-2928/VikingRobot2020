package frc.org.ballardrobotics.types;

import java.util.function.Consumer;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
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

    public void displayOnShuffleboard(String name) {
        displayOnShuffleboard(name, (newValue) -> {});
    }

    public void displayOnShuffleboard(String name, final Consumer<PIDValues> onValueChanged) {        
        int flags = EntryListenerFlags.kNew | EntryListenerFlags.kUpdate;
        var layout = Shuffleboard.getTab("PIDValues").getLayout(name, BuiltInLayouts.kList);

        layout.add("P", this.getP()).withWidget(BuiltInWidgets.kTextView).getEntry().addListener((notif) -> {
            this.setP(notif.value.getDouble());
            onValueChanged.accept(this);
        }, flags);

        layout.add("I", this.getI()).withWidget(BuiltInWidgets.kTextView).getEntry().addListener((notif) -> {
            this.setI(notif.value.getDouble());
            onValueChanged.accept(this);
        }, flags);

        layout.add("D", this.getD()).withWidget(BuiltInWidgets.kTextView).getEntry().addListener((notif) -> {
            this.setD(notif.value.getDouble());
            onValueChanged.accept(this);
        }, flags);

        layout.add("F", this.getF()).withWidget(BuiltInWidgets.kTextView).getEntry().addListener((notif) -> {
            this.setF(notif.value.getDouble());
            onValueChanged.accept(this);
        }, flags);

        layout.add("iZone", this.getIZone()).withWidget(BuiltInWidgets.kTextView).getEntry().addListener((notif) -> {
            this.setIZone(notif.value.getDouble());
            onValueChanged.accept(this);
        }, flags);

        onValueChanged.accept(this);
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
