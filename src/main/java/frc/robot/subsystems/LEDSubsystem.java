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
  private AddressableLEDBuffer buffer1 = new AddressableLEDBuffer(Constants.LEDConstants.LED_Length);
  private int m_rainbowFirstPixelHue = 0;
  private boolean do_the_rainbow = true;
  private int m_bagelbow_first_index = 0;

  public LEDSubsystem() {
    led1.setLength(Constants.LEDConstants.LED_Length);
    led1.setData(buffer1);
    led1.start();
    
  }
  
   public void setColor(Color color) {
    do_the_rainbow = false;
    for(int i = 0; i < buffer1.getLength(); ++i) {
        buffer1.setLED(i, color);
    }
    led1.setData(buffer1);}

    public void DoTheRainbow(boolean val) {
    do_the_rainbow = val;
    
   }

   public void bagelbow() {
    
    LEDScroll(Constants.LEDConstants.LED_1_Length, 0, true);
    LEDScroll(Constants.LEDConstants.LED_2_Length, Constants.LEDConstants.LED_1_Length, false);
    LEDScroll(Constants.LEDConstants.LED_3_Length, Constants.LEDConstants.LED_1_Length + Constants.LEDConstants.LED_2_Length, true);
    LEDScroll(Constants.LEDConstants.LED_4_Length, Constants.LEDConstants.LED_1_Length + Constants.LEDConstants.LED_2_Length + Constants.LEDConstants.LED_3_Length, false);
    m_bagelbow_first_index = (m_bagelbow_first_index + 1);

    led1.setData(buffer1);

   }

   public void LEDScroll(int length, int start, boolean isForwards){//technically doesn't actually scroll
    if(isForwards){
        for(var i = start; i < length/2 + start; i++) {
          buffer1.setLED(((m_bagelbow_first_index%length) + i) % length + start, Color.kBlue);
          buffer1.setLED(((m_bagelbow_first_index%length) + i + length/2) % length + start, Color.kYellow);
        }
    }else{
      for(var i = length + start ; i >= length/2 + start; i--) {
        buffer1.setLED((-m_bagelbow_first_index%length + i) % length + start, Color.kBlue);
        buffer1.setLED((-m_bagelbow_first_index%length + i + length/2) % length + start, Color.kYellow);
      }
    }
   }
 
   public void rainbow(){
    // For every pixel
    for (var i = 0; i < Constants.LEDConstants.LED_Length; i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / Constants.LEDConstants.LED_Length)) % 180;
      // Set the value
      buffer1.setHSV(i, hue, 255, 128);
      //buffer1.setHSV(i, hue, 255, 128);
    }
    
    led1.setData(buffer1);
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
   }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (do_the_rainbow) {
      rainbow();
    }else{
      bagelbow();
    }
  }
}
