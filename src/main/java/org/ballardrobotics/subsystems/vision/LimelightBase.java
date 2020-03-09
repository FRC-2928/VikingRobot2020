package org.ballardrobotics.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * https://docs.limelightvision.io/en/latest/networktables_api.html
 */
public class LimelightBase extends SubsystemBase {
  private static final int kPipelineLED = 0;
  private static final int kForceOffLED = 1;
  private static final int kVisionMode = 0;
  private static final int kDriverMode = 1;

  // Input Entries
  private final NetworkTableEntry m_pipelineEntry;
  private final NetworkTableEntry m_cameraModeEntry;
  private final NetworkTableEntry m_ledEntry;
  
  // Output Entries
  private final NetworkTableEntry m_targetFoundEntry;
  private final NetworkTableEntry m_horizontalAngleEntry;
  private final NetworkTableEntry m_verticalAngleEntry;
  private final NetworkTableEntry m_skewEntry;
  private final NetworkTableEntry m_areaEntry;

  protected final NetworkTable m_table;

  protected boolean m_targetFound;
  protected double m_horizontalAngle;
  protected double m_verticalAngle;
  protected double m_skew;
  protected double m_area;

  public enum CameraMode {
    Driver, Tracking
  }

  public LimelightBase(NetworkTable table) {
    m_table = table;
    m_pipelineEntry = table.getEntry("pipeline");
    m_cameraModeEntry = table.getEntry("camMode");
    m_ledEntry = table.getEntry("ledMode");

    m_targetFoundEntry = table.getEntry("ts");
    m_horizontalAngleEntry = table.getEntry("tx");
    m_verticalAngleEntry = table.getEntry("tx");
    m_skewEntry = table.getEntry("ty");
    m_areaEntry = table.getEntry("ta");
  }

  @Override
  public void periodic() {
    m_targetFound = m_targetFoundEntry.getBoolean(false);
    m_horizontalAngle = m_horizontalAngleEntry.getDouble(0.0);
    m_verticalAngle = m_verticalAngleEntry.getDouble(0.0);
    m_skew = m_skewEntry.getDouble(0.0);
    m_area = m_areaEntry.getDouble(0.0);
  }

  public void setPipeline(int pipeline) {
    m_pipelineEntry.forceSetNumber(pipeline);
  }

  public void setCameraMode(CameraMode mode) {
    if (mode == CameraMode.Tracking) {
      m_cameraModeEntry.forceSetNumber(kVisionMode);
    }
    m_cameraModeEntry.forceSetNumber(kDriverMode);
  }

  public void enableLED() {
    m_ledEntry.forceSetNumber(kPipelineLED);
  }

  public void disableLED() {
    m_ledEntry.forceSetNumber(kForceOffLED);
  }

  public boolean getTargetFound() {
    return m_targetFound;
  }

  public double getHorizontalAngle() {
    return m_horizontalAngle;
  }

  public double getVerticalAngle() {
    return m_verticalAngle;
  }

  public double getSkew() {
    return m_skew;
  }

  public double getArea() {
    return m_area;
  }
}
