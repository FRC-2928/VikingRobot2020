package frc.robot.utilities;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;


public class LEDS {

    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;
    private  int[] m_ledPatternHueBuffer;
    private int bufStart = 0;
    private int hueIndex;
    private int m_rainbowFirstPixelHue = 0;
    private int m_rainbowSecondFirstHue = 230;
    private DriverStation m_driverStation; 
    private DriverStation.Alliance m_allianceColor;
    private boolean firstCallPattern = true;
    private int colorSectionStart = 0;
    private int colorSectionEnd = 10;
    private int m_value = 100;

    public LEDS (){
        m_led = new AddressableLED(9);
    
        // Reuse buffer
        // Default to a length of 60, start empty output
        // only set it once, then just update data
        m_ledBuffer = new AddressableLEDBuffer(60);
        m_led.setLength(m_ledBuffer.getLength());
        
        // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();    
    }

    public enum color {
        RED,BLUE,GREEN,PURPLE,WHITE;
    }

    public void setColor(color color) {

        switch (color) {

            case WHITE:
            for (var i = 0; i < m_ledBuffer.getLength(); i++) {
                // Sets the specified LED to the RGB values for white
                m_ledBuffer.setRGB(i, 255, 255, 255);
             }
             
             m_led.setData(m_ledBuffer);
            break;

            case GREEN:
            for (var i = 0; i < m_ledBuffer.getLength(); i++) {
                // Sets the specified LED to the RGB values for green
                m_ledBuffer.setRGB(i, 0, 255, 0);
             }
             
             m_led.setData(m_ledBuffer);
            break;

            case RED:
            for (var i = 0; i < m_ledBuffer.getLength(); i++) {
                // Sets the specified LED to the RGB values for red
                m_ledBuffer.setRGB(i, 225, 0, 0);
             }
             
             m_led.setData(m_ledBuffer);
            break;

            case BLUE:
            for (var i = 0; i < m_ledBuffer.getLength(); i++) {
                // Sets the specified LED to the RGB values for blue
                m_ledBuffer.setRGB(i, 0, 0, 225);
             }
             
             m_led.setData(m_ledBuffer);
            break;

            case PURPLE:
            for (var i = 0; i < m_ledBuffer.getLength(); i++) {
                // Sets the specified LED to the RGB values for purple
                m_ledBuffer.setRGB(i, 102, 0, 102);
             }
             
             m_led.setData(m_ledBuffer);
            break;


        }
    }
   
    //Sets LEDs to rainbow, repeated calls should "move" the rainbow
    public void rainbow() {
        // For every pixel
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
          // Calculate the hue - hue is easier for rainbows because the color
          // shape is a circle so only one value needs to precess
          final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
          // Set the value
          m_ledBuffer.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        m_rainbowFirstPixelHue += 3;
        // Check bounds
        m_rainbowFirstPixelHue %= 180;
    }
 
        //Sets LEDs to rainbow, repeated calls should "move" the rainbow
    public void movingPattern() {
        int m_rainbowLastPixelHue =0;

        if (firstCallPattern) {  // For every pixel setup initial pattern
           

            m_ledPatternHueBuffer = new int[m_ledBuffer.getLength()];

            // fill first secion of hue buffer
            for (var i = 0; i < (m_ledBuffer.getLength()/4); i++) {
                // Calculate the hue - hue is easier for rainbows because the color
                // TBD how we make pattern
                m_ledPatternHueBuffer[i] =
                    (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
                    m_rainbowLastPixelHue = m_ledPatternHueBuffer[i];
              }

            // fill second secion of hue buffer
            for (var i = (m_ledBuffer.getLength()/4); i < (m_ledBuffer.getLength()/2); i++) {
                // Calculate the hue - hue is easier for rainbows because the color
                // TBD how we make pattern
                m_ledPatternHueBuffer[i] = 
                    m_rainbowLastPixelHue - (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
              }
              
            // fill third secion of hue buffer
            for (var i = (m_ledBuffer.getLength()/2); i < (m_ledBuffer.getLength()*3/4); i++) {
                // Calculate the hue - hue is easier for rainbows because the color
                // TBD how we make pattern
                m_ledPatternHueBuffer[i] =
                    (m_rainbowSecondFirstHue + (i * 180 / m_ledBuffer.getLength())) % 180;
                m_rainbowLastPixelHue = m_ledPatternHueBuffer[i];
              }

            // fill forth secion of hue buffer
            for (var i = (m_ledBuffer.getLength()*3/4); i < (m_ledBuffer.getLength()); i++) {
                // Calculate the hue - hue is easier for rainbows because the color
                // TBD how we make pattern
                m_ledPatternHueBuffer[i] = 
                    m_rainbowLastPixelHue - (m_rainbowSecondFirstHue + (i * 180 / m_ledBuffer.getLength())) % 180;
              }
        }
        
            
        //display pattern
                for (var i = 0; i < m_ledBuffer.getLength(); i++) {
          // Calculate the hue - hue is easier for rainbows because the color
          // shape is a circle so only one value needs to precess
        
                hueIndex = m_ledPatternHueBuffer[ ( (i+bufStart) % m_ledBuffer.getLength() ) ];

                m_ledBuffer.setHSV(i, hueIndex, 255, 128);
        }

        // move pattern one led
        bufStart++;
    }
    // Also call repeatedly to "move"
    private void colorAcross(int hue) {
        m_value = 100;
        // For every pixel
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
          
          if (i < colorSectionEnd && i > colorSectionStart) {
          // Set the section of active LEDs
          m_ledBuffer.setHSV(i, hue, 255, m_value);
          //Dims light as it moves
          m_value -= 5;
        }
        else {
            m_ledBuffer.setHSV(i, 0, 0, 0);
        }
    }
        // Increase by to make the line "move"
        colorSectionStart++;
        colorSectionEnd++;
      
    }
    //call repeatedly for movement 
    public void allianceColor() {
        m_driverStation = DriverStation.getInstance();
        m_allianceColor = m_driverStation.getAlliance();

        switch (m_allianceColor) {
            case Blue:
            colorAcross(240);

            break;

            case Red: 
            colorAcross(0);
            break; 

            case Invalid:
            colorAcross(279);
            break;
        }
    }
}

