package frc.robot.commands.vision;

import org.ballardrobotics.subsystems.vision.LimelightBase;
import org.ballardrobotics.subsystems.vision.LimelightBase.CameraMode;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class LimelightSetTrackingModeCommand extends CommandBase {
  private LimelightBase m_limelight;

  public LimelightSetTrackingModeCommand(LimelightBase limelight) {
    addRequirements(limelight);
    m_limelight = limelight;
  }

  @Override
  public void initialize() {
    m_limelight.enableLED();
    m_limelight.setCameraMode(CameraMode.Tracking);
  }
}
