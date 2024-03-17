package frc.robot.subsystems.lightSubsystem;

import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.utilities.LEDAllocator.RealAllocator;

public class LightIOReal implements LightIO {

  private int length;
  private int startDex;

  private RealAllocator lightIOReal;

  public LightIOReal(RealAllocator lightIOReal, int length) {
    this.length = length;

    this.lightIOReal = lightIOReal;
    startDex = lightIOReal.allocateLength(length);
  }

  @Override
  public void setHSV(int index, int hue, int saturation, int lightness) {
    lightIOReal.setHSV(startDex + index, hue, saturation, lightness);
  }

  @Override
  public int getLength() {
    return length;
  }

  @Override
  public Color8Bit getLED8Bit(int index) {
    return lightIOReal.getColor8Bit(startDex + index);
  }

  @Override
  public void periodic() {
    lightIOReal.periodic();
  }
}
