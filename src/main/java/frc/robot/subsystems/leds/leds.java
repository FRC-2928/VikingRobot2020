package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;
import frc.robot.utilities.LEDS;

public class leds extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  public leds() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    frc.robot.utilities.LEDS.allianceColor    allianceColor() 
  }
}
