package frc.robot.utilities.LEDAllocator;

import edu.wpi.first.wpilibj.util.Color8Bit;

public interface LEDAllocator {
    default int allocateLength(int newLength) {return 0;};

    default void periodic() {};
    
    default Color8Bit getColor8Bit(int index) {return new Color8Bit();};

    default void setHSV(int index, int hue, int saturation, int value) {};
}
