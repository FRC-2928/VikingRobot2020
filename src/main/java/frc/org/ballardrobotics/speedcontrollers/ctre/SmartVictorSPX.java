package frc.org.ballardrobotics.speedcontrollers.ctre;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import frc.org.ballardrobotics.speedcontrollers.SmartSpeedController;

/**
 * Add your docs here.
 */
public class SmartVictorSPX extends WPI_VictorSPX implements SmartSpeedController {
    public static final double kNominalVoltageVolts = 12.0;
    public static final int kVelocitySlotIdx = 0;
    public static final int kPositionSlotIdx = 1;

    private final double kUnitsPerRevolution;

    private ControlMode m_lastControlMode = ControlMode.Disabled;
    private DemandType m_lastDemand1Type = DemandType.Neutral;
    private double m_lastDemand0, m_lastDemand1;

    private double m_targetVelocityRotationsPerSecond;
    private double m_targetPositionRevolutions;
    private double m_targetVoltageVolts;

    public SmartVictorSPX(int deviceNumber) {
        this(deviceNumber, 1.0);
    }

    public SmartVictorSPX(int deviceNumber, double unitsPerRevolution) {
        super(deviceNumber);
        kUnitsPerRevolution = unitsPerRevolution;
        configVoltageCompSaturation(kNominalVoltageVolts);
        enableVoltageCompensation(true);
    }

    @Override
    public void set(double value) {
        setVoltage(value * kNominalVoltageVolts);
    }

    @Override
    public void setVelocity(double velocityRotationsPerSecond) {
        setVelocity(velocityRotationsPerSecond, 0.0);
    }

    @Override
    public void setVelocity(double velocityRotationsPerSecond, double feedforwardVolts) {
        m_targetVelocityRotationsPerSecond = velocityRotationsPerSecond;
        set(ControlMode.Velocity, (velocityRotationsPerSecond * kUnitsPerRevolution) / 10.0, DemandType.ArbitraryFeedForward,
                feedforwardVolts / kNominalVoltageVolts);
    }

    @Override
    public double getMeasuredVelocity() {
        return (getSelectedSensorVelocity() / kUnitsPerRevolution) * 10.0;
    }

    @Override
    public double getTargetVelocity() {
        return m_targetVelocityRotationsPerSecond;
    }

    @Override
    public void setPosition(double positionRotations) {
        setPosition(positionRotations, 0.0);
    }

    @Override
    public void setPosition(double positionRotations, double feedforwardVolts) {
        m_targetPositionRevolutions = positionRotations;
        set(ControlMode.Position, positionRotations * kUnitsPerRevolution, DemandType.ArbitraryFeedForward, feedforwardVolts);
    }

    @Override
    public void setProfiledPosition(double positionRotations) {
        m_targetPositionRevolutions = positionRotations;
        set(ControlMode.MotionMagic, positionRotations * kUnitsPerRevolution);
    }

    @Override
    public double getMeasuredPosition() {
        return (getSelectedSensorPosition() / kUnitsPerRevolution);
    }

    @Override
    public double getTargetPostion() {
        return m_targetPositionRevolutions;
    }

    @Override
    public void setVoltage(double voltageVolts) {
        m_targetVoltageVolts = voltageVolts;
        set(ControlMode.PercentOutput, voltageVolts / kNominalVoltageVolts);
    }

    @Override
    public double getTargetVoltage() {
        return m_targetVoltageVolts;
    }

    @Override
    public double getMeasuredVoltage() {
        return getMotorOutputVoltage();
    }

    @Override
    public void setCurrent(double currentAmps) {
        System.err.println("UnsupportedOperation: VictorSPX does not support setCurrent(double)");
    }

    @Override
    public double getTargetCurrent() {
        System.err.println("UnsupportedOperation: VictorSPX does not support getTargetCurrent()");
        return 0.0;
    }

    @Override
    public double getMeasuredCurrent() {
        System.err.println("UnsupportedOperation: VictorSPX does not support getMeasuredCurrent()");
        return 0.0;
    }

    @Override
    public void set(ControlMode mode, double demand0, DemandType demand1Type, double demand1) {
        if (mode != m_lastControlMode || demand0 != m_lastDemand0 || demand1Type != m_lastDemand1Type
                || m_lastDemand1 != demand1) {
            m_lastControlMode = mode;
            m_lastDemand0 = demand0;
            m_lastDemand1Type = demand1Type;
            m_lastDemand1 = demand1;

            super.set(mode, demand0, demand1Type, demand1);
        } else {
            // Always feed the watchdog.
            feed();
        }
    }

    @Override
    public void setEncoderPosition(double positionRotations) {
        setSelectedSensorPosition((int)(positionRotations * kUnitsPerRevolution));
    }

    @Override
    public void resetEncoder() {
        setSelectedSensorPosition(0);
    }
}
