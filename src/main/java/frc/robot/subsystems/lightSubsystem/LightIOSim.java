package frc.robot.subsystems.lightSubsystem;

import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.utilities.LEDAllocator.SimAllocator;

public class LightIOSim implements LightIO {

  int length = 0;
  int startDex;
  private SimAllocator simAllocator;

  public LightIOSim(SimAllocator simAllocator, int length) {
    this.length = length;

    this.simAllocator = simAllocator;
    startDex = simAllocator.allocateLength(length);
  }

  @Override
  public int getLength() {
    return length;
  }

  @Override
  public Color8Bit getLED8Bit(int index) {
    return simAllocator.getColor8Bit(startDex + index);
  }

  @Override
  public void setHSV(int index, int hue, int saturation, int lightness) {
    simAllocator.setHSV(startDex + index, hue, saturation, hue);
  }

  @Override
  public void periodic() {
    simAllocator.periodic();
  }
}
