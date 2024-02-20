package frc.robot.commands.LightCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LightUnderGlowSubsystem;

public class LightUnderGlowCommand extends Command{
    
    private LightUnderGlowSubsystem lightUnderGlowSubsystem;
    public LightUnderGlowCommand(LightUnderGlowSubsystem lightUnderGlowSubsystem){
        this.lightUnderGlowSubsystem = lightUnderGlowSubsystem;
    }

    @Override
    public void execute(){
        lightUnderGlowSubsystem.beforeMatchColors();
    }

    
}
