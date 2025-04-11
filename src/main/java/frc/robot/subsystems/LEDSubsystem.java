// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LimitSwitch. */
  private AddressableLED led1 = new AddressableLED(Constants.LEDConstants.LED_1_PwmID);
  private AddressableLEDBuffer buffer1 = new AddressableLEDBuffer(Constants.LEDConstants.LED_1_Length);
  private AddressableLED led2 = new AddressableLED(Constants.LEDConstants.LED_2_PwmID);
  private AddressableLEDBuffer buffer2 = new AddressableLEDBuffer(Constants.LEDConstants.LED_2_Length);
  private int m_rainbowFirstPixelHue1 = 0;
  private int m_bagelbow_first_index1 = 0;
  private int m_rainbowFirstPixelHue2 = 0;
  private int m_bagelbow_first_index2 = 0;

  //private boolean do_the_rainbow = true;

  public LEDSubsystem() {
    led1.setLength(Constants.LEDConstants.LED_1_Length);
    led1.setData(buffer1);
    led1.start();
    led2.setLength(Constants.LEDConstants.LED_1_Length);
    led2.setData(buffer1);
    led2.start();
    bagelbow();
  }
  
public void setColor(Color color) {
    //do_the_rainbow = false;
    for(int i = 0; i < buffer1.getLength(); ++i) {
      buffer1.setLED(i, color);
    }
    for(int i = 0; i < buffer2.getLength(); ++i) {
      buffer2.setLED(i, color);
    }
      led1.setData(buffer1);
      led2.setData(buffer2);
    }

    public void DoTheRainbow(boolean val) {
    //do_the_rainbow = val;
    
   }

   public void bagelbow() {
    for(var i = 0; i < 7; i++) {
      buffer1.setLED((m_bagelbow_first_index1 + i) % Constants.LEDConstants.Q1, Color.kBlue);
      buffer1.setLED((m_bagelbow_first_index1 + i) % Constants.LEDConstants.H1, Color.kBlue);
      buffer1.setLED((m_bagelbow_first_index1 + i + 7) % Constants.LEDConstants.Q1, Color.kYellow);
      buffer1.setLED((m_bagelbow_first_index1 + i + 7) % Constants.LEDConstants.H1, Color.kYellow);
    }
    for(var i = 0; i < 7; i++) {
      buffer1.setLED((m_bagelbow_first_index2 + i) % Constants.LEDConstants.Q2, Color.kBlue);
      buffer1.setLED((m_bagelbow_first_index2 + i) % Constants.LEDConstants.H2, Color.kBlue);
      buffer1.setLED((m_bagelbow_first_index2 + i + 7) % Constants.LEDConstants.Q2, Color.kYellow);
      buffer1.setLED((m_bagelbow_first_index2 + i + 7) % Constants.LEDConstants.H2, Color.kYellow);
    }
    m_bagelbow_first_index1 = m_bagelbow_first_index1 + 1 % Constants.LEDConstants.Q1;
    m_bagelbow_first_index2 = m_bagelbow_first_index2 + 1 % Constants.LEDConstants.Q2;

    led1.setData(buffer1);

   }
 
   public void rainbow(){
    // For every pixel
    for (var i = 0; i < Constants.LEDConstants.Q1; i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue1 + (i * 180 / Constants.LEDConstants.Q1)) % 180;
      // Set the value
      buffer1.setHSV(i, hue, 255, 128);
      buffer1.setHSV(i+Constants.LEDConstants.Q1, hue, 255, 128);
    }
    for (var i = 0; i < Constants.LEDConstants.Q2; i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue2 + (i * 180 / Constants.LEDConstants.Q2)) % 180;
      // Set the value
      buffer2.setHSV(i, hue, 255, 128);
      buffer2.setHSV(i+Constants.LEDConstants.Q2, hue, 255, 128);
    }
    led1.setData(buffer1);
    led2.setData(buffer1);
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue1 += 3;
    // Check bounds
    m_rainbowFirstPixelHue1 %= 180;
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue2 += 3;
    // Check bounds
    m_rainbowFirstPixelHue2 %= 180;
   }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
      bagelbow();
  }
}
