package frc.robot.subsystems.lightSubsystem;

import edu.wpi.first.wpilibj.util.Color8Bit;

public interface LightIO {
  // AddressableLEDBuffer getBuffer();

  default void setHSV(int index, int hue, int saturation, int lightness) {}
  ;

  default int getLength() {
    return 0;
  }
  ;

  default Color8Bit getLED8Bit(int index) {
    return new Color8Bit();
  }
  ;

  default void periodic() {}
  ;
}
