
package frc.utility.vanity;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class AddressableLEDs
{
    // PWM port 9
    // Must be a PWM header, not MXP or DIO


    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    public AddressableLED m_led;
    public AddressableLEDBuffer m_ledBuffer;

    public AddressableLEDs()
    {
        m_led= new AddressableLED(6);
        m_ledBuffer = new AddressableLEDBuffer(150);
        m_led.setLength(m_ledBuffer.getLength());

            // Set the data
            m_led.setData(m_ledBuffer);
            m_led.start();
    }

    public void setLED()
    {
        for (var i = 0; i < m_ledBuffer.getLength();i++) {
            // Sets the specified LED to the RGB values for red
            m_ledBuffer.setRGB(i, 255, 69, 0);
         }

         m_led.setData(m_ledBuffer);
    }


}