package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.LightIndividualSubsystem;

public class LightCommandGroup {
    
    public static Command blankOn(LightIndividualSubsystem lightIndividualSubsystem){
        return new  SequentialCommandGroup (
                    new LightBlankCommand(lightIndividualSubsystem).withTimeout(3),
                    new WaitCommand(5),
                    new LightsOnCommand(lightIndividualSubsystem).withTimeout(3));

    }
}
