package frc.robot.utilities;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.lightSubsystem.LightIO;
import frc.robot.subsystems.lightSubsystem.LightIOReal;
import frc.robot.subsystems.lightSubsystem.LightIOSim;
import frc.robot.subsystems.lightSubsystem.LightSubsystem;
import frc.robot.utilities.LEDAllocator.SimAllocator;

public final class LEDSubsystemFactory {

    public static LightSubsystem lightIndividualSubsystem(int port, int length) {
        return new LightSubsystem(
                switch (Constants.robotMode) {
                    case REAL -> new LightIOReal(port, length);
                    case SIM -> new LightIOSim(new SimAllocator(), length);
                    default -> new LightIO() {};
                });
    }
}
