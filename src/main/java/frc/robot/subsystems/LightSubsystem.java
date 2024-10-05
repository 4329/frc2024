package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightSubsystem extends SubsystemBase {

  SerialPort serialPort;
  LEDPattern currentPattern;

  public static class State {
    private static boolean shooting;
    private static boolean climbing;
    private static boolean outdexing;
    private static boolean intaking;
    private static boolean hasNote;
    private static boolean sourcing;
    private static boolean inRange;
    private static boolean isConnected;

    protected static boolean isShooting() {
      return shooting;
    }
    public static void setShooting(boolean shooting) {
      State.shooting = shooting;
    }
    protected static boolean isClimbing() {
      return climbing;
    }
    public static void setClimbing(boolean climbing) {
      State.climbing = climbing;
    }
    protected static boolean isOutdexing() {
      return outdexing;
    }
    public static void setOutdexing(boolean outdexing) {
      State.outdexing = outdexing;
    }
    protected static boolean isIntaking() {
      return intaking;
    }
    public static void setIntaking(boolean intaking) {
      State.intaking = intaking;
    }
    protected static boolean isHasNote() {
      return hasNote;
    }
    public static void setHasNote(boolean hasNote) {
      State.hasNote = hasNote;
    }
    protected static boolean isSourcing() {
      return sourcing;
    }
    public static void setSourcing(boolean sourcing) {
      State.sourcing = sourcing;
    }
    protected static boolean isInRange() {
      return inRange;
    }
    public static void setInRange(boolean inRange) {
      State.inRange = inRange;
    }
    protected static boolean isConnected() {
      return isConnected;
    }
    public static void setIsConnected(boolean isConnected) {
      State.isConnected = isConnected;
    }
  }

  private class Node {
    public LEDPattern pattern;
    public List<Edge> edges;

    public Node(LEDPattern pattern, List<Edge> edges) {
      this.pattern = pattern;
      this.edges = edges;
    }

    public void addEdge(Edge toAdd) {
      edges.add(toAdd);
    }

    public void evalEdges() {
      edges.forEach((x) -> {
        if (x.whenGo.get()) {
          current = x.next;
          setLEDPattern(current.pattern);
        }
      });
    }
  }
  private class Edge {
    public Node next;
    public Supplier<Boolean> whenGo;

    public Edge(Node next, Supplier<Boolean> whenGo) {
      this.next = next;
      this.whenGo = whenGo;
    }
  }
  private Node current;
  private Timer timer = new Timer();

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
  
    setUpGraph();
    timer.start();
  }

  private void setUpGraph() {
    Node head = new Node(LEDPattern.NOTHING, new ArrayList<>());
    current = head;
    setLEDPattern(head.pattern);

    head.addEdge(new Edge(new Node(LEDPattern.ORANGE, new ArrayList<>()), () -> {
      return timer.hasElapsed(10);
    }));
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
    current.evalEdges();
  }
}
