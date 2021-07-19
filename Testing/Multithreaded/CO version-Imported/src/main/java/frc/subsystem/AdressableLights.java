package frc.subsystem;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.Constants.MiscConstants;

public class AdressableLights {
            public AddressableLED m_led;
            public AddressableLEDBuffer m_ledBuffer;
        
    public AdressableLights() {
        m_led= new AddressableLED(MiscConstants.LED);
        m_ledBuffer= new AddressableLEDBuffer(150);
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

  
    public void setLED() {
        int j = 1;
        for (var i = 0; i < m_ledBuffer.getLength(); i+=2) {
            
            m_ledBuffer.setRGB(i, 255, 69, 0);
            m_ledBuffer.setRGB(j, 255, 255, 255);
            j+=2;
         }
         
         m_led.setData(m_ledBuffer);
    }
   

}
