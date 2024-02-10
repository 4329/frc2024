package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.LightIndividualSubsystem;

public class LightCommandGroup {
    
    public static Command blankOn(LightIndividualSubsystem lightIndividualSubsystem){
        return new ParallelCommandGroup (
                    new LightBlankCommand(lightIndividualSubsystem).withTimeout(3),
                    new LightsOnCommand(lightIndividualSubsystem).withTimeout(3));

    }
}
