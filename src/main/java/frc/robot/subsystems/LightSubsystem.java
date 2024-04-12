package frc.robot.subsystems;

import javax.sql.rowset.serial.SerialException;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightSubsystem extends SubsystemBase {

  SerialPort serialPort;

  public LightSubsystem() {
    try {
      serialPort = new SerialPort(9600, Port.kUSB1);
    } catch (Exception e) {
      serialPort = new SerialPort(9600, Port.kUSB2);
    }
  }

  public enum LEDPattern {
    BLUE,
    CORAL
  }

  public void setLEDPattern(LEDPattern lPattern) {
    byte[] bytey = ((lPattern.ordinal() + 1) + "\n").getBytes();
    serialPort.write(bytey, bytey.length);
  }

  @Override
  public void periodic() {
    System.out.println(serialPort.readString());
    setLEDPattern(LEDPattern.values()[(int)(Math.random() * 2)]);
  }
}
