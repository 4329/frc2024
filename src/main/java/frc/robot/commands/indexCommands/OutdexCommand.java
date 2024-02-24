package frc.robot.commands.indexCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class OutdexCommand extends Command {

    private IndexSubsystem indexSubsystem;
    
    
    
    public OutdexCommand(IndexSubsystem indexSubsystem) {
        this.indexSubsystem = indexSubsystem;
        addRequirements(indexSubsystem);


    }



 
    @Override
    public void initialize() {
      
    }


    @Override
    public void execute() {

        System.out.println("index out");
        indexSubsystem.out();

    }


    @Override
    public void end(boolean interrupted) {
        indexSubsystem.stop();

        System.out.println("index out stop");
     
    }

    @Override
    public boolean isFinished() {
       return false;
    }


    
    
}
