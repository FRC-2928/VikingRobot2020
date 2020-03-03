package org.ballardrobotics.subsystems.vision;

import org.ballardrobotics.types.LimelightData;

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
  private final NetworkTableEntry m_xAngleEntry;
  private final NetworkTableEntry m_yAngleEntry;
  private final NetworkTableEntry m_skewEntry;
  private final NetworkTableEntry m_areaEntry;

  protected final LimelightData m_data;
  protected final NetworkTable m_table;

  public enum CameraMode {
    Driver, Tracking
  }

  public LimelightBase(NetworkTable table) {
    m_table = table;
    m_pipelineEntry = table.getEntry("pipeline");
    m_cameraModeEntry = table.getEntry("camMode");
    m_ledEntry = table.getEntry("ledMode");

    m_targetFoundEntry = table.getEntry("ts");
    m_xAngleEntry = table.getEntry("tx");
    m_yAngleEntry = table.getEntry("tx");
    m_skewEntry = table.getEntry("ty");
    m_areaEntry = table.getEntry("ta");
    
    m_data = new LimelightData();
  }

  @Override
  public void periodic() {
    m_data.setTargetFound(m_targetFoundEntry.getBoolean(false));
    m_data.setHorizontalAngle(m_xAngleEntry.getDouble(0.0));
    m_data.setVerticalAngle(m_yAngleEntry.getDouble(0.0));
    m_data.setSkew(m_skewEntry.getDouble(0.0));
    m_data.setArea(m_areaEntry.getDouble(0.0));
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

  public LimelightData getData() {
    return m_data;
  }
}
