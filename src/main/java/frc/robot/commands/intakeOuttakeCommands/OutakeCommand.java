package frc.robot.commands.intakeOuttakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class OutakeCommand extends Command {

    private IntakeSubsystem intakeSubsystem;
    
    
    
    public OutakeCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);


    }

 
    @Override
    public void initialize() {
      
    }


    @Override
    public void execute() {

        System.out.println("intake out");
        intakeSubsystem.out();

    }


    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stop();

        System.out.println("intake out stop");
     
    }

    @Override
    public boolean isFinished() {
       return false;
    }


    
    
}
