package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootSubsystem;

public class CommandGroups {


    public static Command intakeFull(IntakeSubsystem intakeSubsystem, IndexSubsystem indexSubsystem) {


        return new ParallelCommandGroup(

                new IntakeCommand(intakeSubsystem),
                new IndexCommand(indexSubsystem)

        );

    }

    public static Command outakeFull(IntakeSubsystem intakeSubsystem, IndexSubsystem indexSubsystem) {

        return new ParallelCommandGroup (


            new OutakeCommand(intakeSubsystem),
            new OutdexCommand(indexSubsystem)


        );

    }


    public static Command releaseToShoot(ShootSubsystem shootSubsystem, IndexSubsystem indexSubsystem) {


            return new ParallelCommandGroup (
                    new IndexHoldCommand(indexSubsystem),
                    new ShootCommand(shootSubsystem).withTimeout(5)).withTimeout(7);


        
    }

}
    