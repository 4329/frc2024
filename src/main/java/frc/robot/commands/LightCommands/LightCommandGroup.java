package frc.robot.commands.LightCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.LightBlankCommand;
import frc.robot.commands.LightsOnCommand;
import frc.robot.subsystems.LightIndividualSubsystem;

public class LightCommandGroup {

    public static Command blankOn(LightIndividualSubsystem lightIndividualSubsystem) {
        return new SequentialCommandGroup(
                new LightBlankCommand(lightIndividualSubsystem).withTimeout(3),
                new WaitCommand(5),
                new LightsOnCommand(lightIndividualSubsystem).withTimeout(3));
    }

    public static Command slowProgress(LightIndividualSubsystem lightIndividualSubsystem) {
        SequentialCommandGroup sequentialCommandGroup = new SequentialCommandGroup();
        // new LightBlackCommand(lightIndividualSubsystem).withTimeout(2),
        // new WaitCommand(1),
        // new LightProgressCommand(lightIndividualSubsystem,
        // 0.6,255,255).withTimeout(3),
        // new WaitCommand(5)
        for (int i = 1; i <= 60; i ++) {
            sequentialCommandGroup.addCommands(
                    new LightBlackCommand(lightIndividualSubsystem).withTimeout(0.1),

                    new WaitCommand(0.1),
                    //start, int stop, int hue, int saturation
                    new LoadingBarCommand(lightIndividualSubsystem, i, i+3, 60, 60).withTimeout(0.1),
                    new WaitCommand(0.5));
            // if (i > 0.1) {
            //     sequentialCommandGroup.addCommands(
            //             new LightProgressCommand(lightIndividualSubsystem, i-0.1, 0, 0).withTimeout(0.1));
            // }

        }
        return sequentialCommandGroup;
    }
}
