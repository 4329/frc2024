package frc.robot.utilities.LEDAllocator;

public interface LEDAllocator {
    default int allocateLength(int length) {return 0;};

    default void periodic() {};
}
