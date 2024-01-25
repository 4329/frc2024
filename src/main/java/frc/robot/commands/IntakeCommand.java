package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {

    private IntakeSubsystem intakeSubsystem;
    private IndexSubsystem indexSubsystem;
    
    
    
    public IntakeCommand(IntakeSubsystem intakeSubsystem, IndexSubsystem indexSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.indexSubsystem = indexSubsystem;
        addRequirements(intakeSubsystem, indexSubsystem);


    }



 
    @Override
    public void initialize() {
      
    }


    @Override
    public void execute() {

        System.out.println("is running 324098");
        intakeSubsystem.in();
        indexSubsystem.in();

    }


    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stop();
        indexSubsystem.stop();

        System.out.println("is stopping 9876543");
     
    }

    @Override
    public boolean isFinished() {
       return false;
    }


    
    
}
