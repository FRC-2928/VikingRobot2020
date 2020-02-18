package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.LEDS;
import frc.robot.utilities.LEDS.color;



public class LEDSubsystem extends SubsystemBase {
  
   private final LEDS m_LEDS;
   

   public enum Patterns {
       RAINBOW,ALLAINCE,PATTERN
   }

    private Patterns m_pattern;

  public LEDSubsystem(Patterns patterns) {
    m_pattern = patterns;
    m_LEDS = new LEDS();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch (m_pattern) {
        case ALLAINCE:
        m_LEDS.allianceColor();
        break;

        case RAINBOW:
        m_LEDS.rainbow();
        break;

        case PATTERN:
        m_LEDS.movingPattern();
        break;
    }
    
  }

  //still needs to be called repeatedly to work
  
  public void setPattern(Patterns patterns) {
      m_pattern = patterns; 
  }

  public void SetColor(color color) {
    m_LEDS.setColor(color);
  }
}
