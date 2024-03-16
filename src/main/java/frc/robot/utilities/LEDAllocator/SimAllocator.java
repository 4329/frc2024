package frc.robot.utilities.LEDAllocator;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj.util.Color8Bit;

public final class SimAllocator implements LEDAllocator {

  private AddressableLEDSim addressableLEDSim;
  private AddressableLED addressableLED;
  private AddressableLEDBuffer addressableLEDBuffer;

  private int length;

  public SimAllocator() {
    addressableLEDBuffer = new AddressableLEDBuffer(0);
    addressableLED = new AddressableLED(1);
    addressableLED.start();
    addressableLEDSim = AddressableLEDSim.createForChannel(1);
    addressableLEDSim.setOutputPort(1);
    addressableLEDSim.setLength(addressableLEDBuffer.getLength());
    addressableLEDSim.setInitialized(true);
    addressableLEDSim.setRunning(true);
  }

  @Override
  public int allocateLength(int newLength) {
    addressableLEDSim.setRunning(false);
    if (length != 0) {
      length += 10;
    }
    int storeLength = length;
    length += newLength;
    Color8Bit[] transfer = new Color8Bit[addressableLEDBuffer.getLength()];
    for (int i = 0; i < addressableLEDBuffer.getLength(); i++) {
      transfer[i] = addressableLEDBuffer.getLED8Bit(i);
    }
    addressableLEDBuffer = new AddressableLEDBuffer(length);
    for (int i = 0; i < transfer.length; i++) {
      addressableLEDBuffer.setLED(i, transfer[i]);
    }

    addressableLEDSim.setLength(addressableLEDBuffer.getLength());
    addressableLEDSim.setRunning(true);
    return storeLength;
  }

  public Color8Bit getColor8Bit(int index) {
    return addressableLEDBuffer.getLED8Bit(index);
  }

  public void setHSV(int index, int hue, int saturation, int value) {
    addressableLEDBuffer.setHSV(index, hue, saturation, value);
  }

  public void periodic() {
    addressableLED.setData(addressableLEDBuffer);
  }
}
