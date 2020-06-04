package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class AdressableLights
    {
            public AddressableLED m_led= new AddressableLED(6); //placeholder
            public AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(150);
        
    public AdressableLights()
    {
    
        m_led.setLength(m_ledBuffer.getLength());
            
            // Set the data
            m_led.setData(m_ledBuffer);
            m_led.start();
    }

   /* public void init()
    {
          
    }
    */
    public void setLED()
    {
        int j = 1;
        for (var i = 0; i < m_ledBuffer.getLength(); i+=2) {
            
            m_ledBuffer.setRGB(i, 255, 69, 0);
            m_ledBuffer.setRGB(j, 255, 255, 255);
            j+=2;
         }
         
         m_led.setData(m_ledBuffer);
    }
   

}