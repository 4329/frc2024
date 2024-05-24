package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class LightSubsystem extends SubsystemBase {

  SerialPort serialPort;
  LEDPattern currentPattern;

  public static class State {
    public static boolean shooting;
    public static boolean climbing;
    public static boolean outdexing;
    public static boolean intaking;
    public static boolean hasNote;
    public static boolean sourcing;
    public static boolean inRange;

    public static boolean isShooting() {
      return shooting;
    }
    public static void setShooting(boolean shooting) {
      State.shooting = shooting;
    }
    public static boolean isClimbing() {
      return climbing;
    }
    public static void setClimbing(boolean climbing) {
      State.climbing = climbing;
    }
    public static boolean isOutdexing() {
      return outdexing;
    }
    public static void setOutdexing(boolean outdexing) {
      State.outdexing = outdexing;
    }
    public static boolean isIntaking() {
      return intaking;
    }
    public static void setIntaking(boolean intaking) {
      State.intaking = intaking;
    }
    public static boolean isHasNote() {
      return hasNote;
    }
    public static void setHasNote(boolean hasNote) {
      State.hasNote = hasNote;
    }
    public static boolean isSourcing() {
      return sourcing;
    }
    public static void setSourcing(boolean sourcing) {
      State.sourcing = sourcing;
    }
    public static boolean isInRange() {
      return inRange;
    }
    public static void setInRange(boolean inRange) {
      State.inRange = inRange;
    }
  }

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
    MAGENTA,
    GREEN,
    YELLOW,
    RED,
    ORANGE,
    NOTHING,
    ALRED,
    ALBLUE,
    CLIMB
  }

  public void setLEDPattern(LEDPattern lPattern) {
    if (serialPort != null) {
      byte[] bytey = (lPattern.ordinal() + "\n").getBytes();
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

  @Override
  public void periodic() {
    
  }
}
