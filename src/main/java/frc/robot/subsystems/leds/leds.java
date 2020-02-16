package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;
import frc.robot.utilities.LEDS;



public class leds extends SubsystemBase {
  
   private final LEDS m_LEDS;
   
  public leds() {

    m_LEDS = new LEDS();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_LEDS.allianceColor();
  }

  //still needs to be called repeatedly to work
  @Override
  public void displayRainbow() {
      m_LEDS.rainbow();
  }
}
