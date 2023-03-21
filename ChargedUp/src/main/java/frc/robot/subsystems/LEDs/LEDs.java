// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDs;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
  AddressableLED m_led;
  AddressableLEDBuffer m_ledBuffer;
  int m_rainbowFirstPixelHue = 0;
  
  int mode = 0;
  /* 
  0 = Solid Color;
  1 = Rainbow
  2 = Noise
  */

  OpenSimplex noise;
  double height = 20;
  double width = 600;
  double xscale = 0.015;
  double yscale = 0.05;
  double offset = 0.01;
  double offsetInc = 0.01;
  double[] hueRange = {85,120};
  double[] noiseRange = {-1,1};

  Timer timer = new Timer();

  /** Creates a new LEDs. */
  public LEDs() {
    m_led = new AddressableLED(6);

    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    m_ledBuffer = new AddressableLEDBuffer(150);
    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();

    noise = new OpenSimplex();
  }

  public void SetModeSolid(int hue){
    mode = 0;
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setHSV(i, hue, 255, 128);
    }
  }

  public void SetModeRainbow(){
    mode = 1;
  }

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

  private void SetModeNoise(double[] hueRange){
    this.hueRange = hueRange;
    this.mode = 2;
  }

  private void noise(){
    for (var x = 0; x < m_ledBuffer.getLength(); x++) {
      double noiseVal = noise.noise2_ImproveX(1934173838, x, offset);
      int hue = (int)Remap(noiseVal, noiseRange, hueRange) % 180;
      m_ledBuffer.setHSV(x, hue, 255, 128);
    }
    offset += offsetInc;
  }

  private double Remap(double startVal, double[] startRange, double[] endRange){
    return (startVal-startRange[0]) * (endRange[1]-endRange[0]) / (startRange[1]- startRange[0]) + endRange[0];
  }
  
  private double Lerp(double start, double end, double lerpVal){
    return (end-start) * lerpVal + start;
  }

  @Override
  public void periodic() {
    if(mode == 0){return;}
    else if(mode == 1)
    {
      rainbow();
      m_led.setData(m_ledBuffer);
    }
    else if (mode == 2)
    {
      noise();
      m_led.setData(m_ledBuffer);
    }
    // This method will be called once per scheduler run
  }
}
