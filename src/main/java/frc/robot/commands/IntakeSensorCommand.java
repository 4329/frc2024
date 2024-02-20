package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LineBreakSensorSubsystem;

public class IntakeSensorCommand extends Command {

    private IntakeSubsystem intakeSubsystem;
    private LineBreakSensorSubsystem lineBreakSensorSubsystem;
    
    
    public IntakeSensorCommand(IntakeSubsystem intakeSubsystem, LineBreakSensorSubsystem lineBreakSensorSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.lineBreakSensorSubsystem = lineBreakSensorSubsystem;
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
         return !lineBreakSensorSubsystem.isNotBroken();
    }


    
    
}
