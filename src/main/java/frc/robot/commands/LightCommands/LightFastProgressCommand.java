package frc.robot.commands.LightCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LightIndividualSubsystem;

public class LightFastProgressCommand extends Command{
    private int count = 0;
    private int countAlter = 1;
    private LightIndividualSubsystem lightIndividualSubsystem;

    public LightFastProgressCommand(LightIndividualSubsystem lightIndividualSubsystem){
        this.lightIndividualSubsystem = lightIndividualSubsystem;
        count = 0;
        addRequirements(lightIndividualSubsystem);
    }

    @Override
    public void execute() {
        count+=countAlter;
        if(count>60){
            countAlter = -1;
        }
        else if(count < 0){
            countAlter = 1;
        }
        lightIndividualSubsystem.loadingBarColor(count, count+6, 60, 60);
    }

    // @Override
    // public boolean isFinished() {
    //     return true;
    // }
    @Override
    public void end(boolean interrupted) {
        lightIndividualSubsystem.beforeMatchColors();
    }
}
