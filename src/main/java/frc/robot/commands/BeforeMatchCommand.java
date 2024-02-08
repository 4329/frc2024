package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LightIndividualSubsystem;

public class BeforeMatchCommand extends Command{
    
    public LightIndividualSubsystem lightIndividualSubsystem;

    public BeforeMatchCommand(LightIndividualSubsystem lightIndividualSubsystem){
        this.lightIndividualSubsystem = lightIndividualSubsystem;
        addRequirements(lightIndividualSubsystem);
    }
    
    @Override
    public void execute() {
        lightIndividualSubsystem.beforeMatchColors();
    }

}
