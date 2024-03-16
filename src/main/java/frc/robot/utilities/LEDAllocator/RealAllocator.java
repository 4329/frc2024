package frc.robot.utilities.LEDAllocator;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class RealAllocator implements LEDAllocator {

  private AddressableLED addressableLED;
  private AddressableLEDBuffer addressableLEDBuffer;

  private int length;

  public RealAllocator() {
    addressableLED = new AddressableLED(1);
    addressableLEDBuffer = new AddressableLEDBuffer(0);
    addressableLED.start();
  }

  @Override
  public int allocateLength(int newLength) {
    addressableLED.stop();

    int storeLength = length;
    length += newLength;

    addressableLEDBuffer = new AddressableLEDBuffer(length);
    if (storeLength > 0) {
      Color8Bit[] transfer = new Color8Bit[storeLength];
      for (int i = 0; i < storeLength; i++) {
        transfer[i] = addressableLEDBuffer.getLED8Bit(i);
      }

      for (int i = 0; i < storeLength; i++) {
        addressableLEDBuffer.setLED(i, transfer[i]);
      }
    }
    addressableLED.setLength(length);

    addressableLED.start();
    return storeLength;
  }

  @Override
  public Color8Bit getColor8Bit(int index) {
    return addressableLEDBuffer.getLED8Bit(index);
  }

  @Override
  public void setHSV(int index, int hue, int saturation, int value) {
    addressableLEDBuffer.setHSV(index, hue, saturation, value);
  }

  @Override
  public void periodic() {
    addressableLED.setData(addressableLEDBuffer);
  }
}
