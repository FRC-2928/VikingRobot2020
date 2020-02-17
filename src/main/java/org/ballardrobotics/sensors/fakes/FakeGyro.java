package org.ballardrobotics.sensors.fakes;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.interfaces.Gyro;

/**
 * Add your docs here.
 */
public class FakeGyro implements Gyro {
    public DoubleSupplier m_angleSupplier, m_rateSupplier;
    public Runnable m_onClose, m_onCalibrate, m_onReset;

    public FakeGyro() {
        this(() -> 0, () -> 0);
    }

    public FakeGyro(DoubleSupplier angleSupplier, DoubleSupplier rateSupplier) {
        this(angleSupplier, rateSupplier, () -> {}, () -> {}, () -> {});
    }

    public FakeGyro(DoubleSupplier angleSupplier, DoubleSupplier rateSupplier, Runnable onClose, Runnable onCalibrate, Runnable onReset) {
        m_angleSupplier = angleSupplier;
        m_rateSupplier = rateSupplier;
        m_onClose = onClose;
        m_onCalibrate = onCalibrate;
        m_onReset = onReset;
    }

    @Override
    public void close() throws Exception {
        m_onClose.run();
    }

    @Override
    public void calibrate() {
        m_onCalibrate.run();
    }

    @Override
    public void reset() {
        m_onReset.run();
    }

    @Override
    public double getAngle() {
        return m_angleSupplier.getAsDouble();
    }

    @Override
    public double getRate() {
        return m_rateSupplier.getAsDouble();
    }

    public void setAngleSupplier(DoubleSupplier angleSupplier) {
        m_angleSupplier = angleSupplier;
    }

    public void setRateSupplier(DoubleSupplier rateSupplier) {
        m_rateSupplier = rateSupplier;
    }

    public void setOnClose(Runnable onClose) {
        m_onClose = onClose;
    }

    public void setOnCalibrate(Runnable onCalibrate) {
        m_onCalibrate = onCalibrate;
    }

    public void setOnReset(Runnable onReset) {
        m_onReset = onReset;
    }
}
