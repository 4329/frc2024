package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightSubsystem extends SubsystemBase {

  SerialPort serialPort;

  public LightSubsystem() {
    serialPort = new SerialPort(9600, Port.kUSB);
  }

  public enum LEDPattern {
    BLUE,
    RED,
    BLANCHED_ALMOND,
    KHAKI,
    DARK_SALMON
  }

  public void setLEDPattern(LEDPattern lPattern) {
    byte[] bytey = (lPattern.ordinal() + "\n").getBytes();
    serialPort.write(bytey, bytey.length);
  }

  @Override
  public void periodic() {
    System.out.println(serialPort.readString());
    setLEDPattern(LEDPattern.DARK_SALMON);
  }
}
