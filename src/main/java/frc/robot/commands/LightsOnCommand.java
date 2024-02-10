package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LightIndividualSubsystem;

public class LightsOnCommand extends Command{
    LightIndividualSubsystem lightIndividualSubsystem;

    public LightsOnCommand(LightIndividualSubsystem lightIndividualSubsystem){
        this.lightIndividualSubsystem = lightIndividualSubsystem;
        addRequirements(lightIndividualSubsystem);
    }

    @Override
    public void initialize() {
        lightIndividualSubsystem.blank();
        
        lightIndividualSubsystem.lightsOn();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
