package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class LightSubsystem extends SubsystemBase {

  SerialPort serialPort;
  LEDPattern currentPattern;

  public LightSubsystem() {
    int count = 0;
    try {
      serialPort = new SerialPort(9600, Port.kUSB1);
    } catch (Exception e) {
      count++;
    }
    try {
      serialPort = new SerialPort(9600, Port.kUSB2);
    } catch (Exception e) {
      count++;
    }

    if (count == 2) Logger.recordOutput("Lights", "no :(");
    else Logger.recordOutput("Lights", "Yes!");
  }

  public enum LEDPattern {
    BLUE,
    CYAN,
    GREEN,
    YELLOW,
    RED,
    NOTHING
  }

  public void setLEDPattern(LEDPattern lPattern) {
    if (currentPattern != null) {
      byte[] bytey = ((lPattern.ordinal() + 1) + "\n").getBytes();
      serialPort.write(bytey, bytey.length);

      currentPattern = lPattern;
    } else {
      System.out.println("No USB");
    }
    Logger.recordOutput("Current pattern", lPattern);
  }

  public LEDPattern getLEDPattern() {
    return currentPattern;
  }
}
