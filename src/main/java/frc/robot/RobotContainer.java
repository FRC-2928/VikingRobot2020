package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.subsystems.Subsystems;

public class RobotContainer {
  private final Subsystems m_subsystems;
  private final Compressor m_compressor;

  public RobotContainer() {
    m_subsystems = new Subsystems();
    m_compressor = new Compressor();

    var startCompressor = new InstantCommand(() -> m_compressor.start());
    var stopCompressor = new InstantCommand(() -> m_compressor.stop());
    var layout = Shuffleboard.getTab("Pneumatics").getLayout("Compressor", BuiltInLayouts.kList).withSize(2, 2).withPosition(0, 0);
    layout.add("start", startCompressor);
    layout.add("stop", stopCompressor);
    layout.addBoolean("enabled", m_compressor::enabled);
    startCompressor.schedule();
  }

  public void configureButtonBindings() {

  }

  public Command getAutonomousCommand() {
    return new PrintCommand("auto not implemented");
  }
}
