package frc.robot.utilities;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;


public class LEDS {

    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;
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
                // Sets the specified LED to the RGB values for red
                m_ledBuffer.setRGB(i, 255, 255, 255);
             }
             
             m_led.setData(m_ledBuffer);
            break;

            case GREEN:
            for (var i = 0; i < m_ledBuffer.getLength(); i++) {
                // Sets the specified LED to the RGB values for red
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
                // Sets the specified LED to the RGB values for red
                m_ledBuffer.setRGB(i, 225, 0, 0);
             }
             
             m_led.setData(m_ledBuffer);
            break;

            case PURPLE:
            for (var i = 0; i < m_ledBuffer.getLength(); i++) {
                // Sets the specified LED to the RGB values for red
                m_ledBuffer.setRGB(i, 225, 0, 0);
             }
             
             m_led.setData(m_ledBuffer);
            break;


        }
    }

 
  
}

