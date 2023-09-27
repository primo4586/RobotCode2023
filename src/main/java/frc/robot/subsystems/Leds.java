// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Leds extends SubsystemBase {
  private AddressableLED m_led;
  private static AddressableLEDBuffer m_ledBuffer;

  /** Creates a new Leds. */
  public Leds(int port, int length) {
    m_led = new AddressableLED(port);
    m_ledBuffer = new AddressableLEDBuffer(length);

    m_led.setLength(m_ledBuffer.getLength());
    
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public enum ColorsRGBLeds {
    //Primo colors
    PRIMO_ORANGE(12, 32, 43),
    PRIMO_BLUE(43, 34, 43),

    //Rainbow RGB
    RED(255, 0, 0),
    GREEN(0, 255, 0),
    BLUE(0, 0, 255),
    ORANGE(255, 165, 0),
    YELLOW(255, 255, 0),
    INDIGO(75, 0, 130),
    VIOLET(148, 0, 211),

    //Extreme Colors
    WHITE(255, 255, 255),
    OFF(0, 0, 0);
    
    final int red;
    final int green;
    final int blue;

    ColorsRGBLeds (int red, int green, int blue){
        this.red = red;
        this.green = green;
        this.blue = blue;
    }
  }
    

  //Color setters
  public void setLedsColor (int red, int green, int blue){
    for (var i = 0; i < m_ledBuffer.getLength(); i++) 
      m_ledBuffer.setRGB(i, red, green, blue);

    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public void setLedsColor (ColorsRGBLeds color){
    setLedsColor(color.red, color.green, color.blue);
  }

  public void turnOffLed() {
    setLedsColor(ColorsRGBLeds.OFF);
  }

  //Getters
  public int getLength(){
    return m_ledBuffer.getLength();
  }

    int m_rainbowFirstPixelHue = 0;
  //Just fun activities
  private void rainbow() {
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

  public void primoBlinking() {
    while(true){
      setLedsColor(ColorsRGBLeds.PRIMO_BLUE);
      Timer.delay(0.5);
      setLedsColor(ColorsRGBLeds.PRIMO_ORANGE);
      Timer.delay(0.5);
    }
  }

  public void primoDots() {
    while(true){
      for (int i = 0; i < m_ledBuffer.getLength(); i += 2) {
        m_ledBuffer.setRGB(i, 23, 23, 23);
        m_ledBuffer.setRGB(i + 1, 23, 23, 23);
      }
      Timer.delay(0.5);
      for (int i = 0; i < m_ledBuffer.getLength(); i += 2) {
        m_ledBuffer.setRGB(i, 23, 23, 23);
        m_ledBuffer.setRGB(i + 1, 23, 23, 23);
      }
      Timer.delay(0.5);
    }
  }
  

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_led.setData(m_ledBuffer);
    m_led.start();
  }
}
