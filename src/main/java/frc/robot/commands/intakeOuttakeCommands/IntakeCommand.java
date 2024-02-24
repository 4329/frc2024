package frc.robot.commands.intakeOuttakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {

    private IntakeSubsystem intakeSubsystem;
    
    
    
    public IntakeCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);


    }



 
    @Override
    public void initialize() {
      
    }


    @Override
    public void execute() {

        System.out.println("intake in");
        intakeSubsystem.in();

    }


    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stop();

        System.out.println("intake in stop");
     
    }

    @Override
    public boolean isFinished() {
       return false;
    }


    
    
}
