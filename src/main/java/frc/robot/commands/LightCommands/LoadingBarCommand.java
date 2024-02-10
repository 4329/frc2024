package frc.robot.commands.LightCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LightIndividualSubsystem;

public class LoadingBarCommand extends Command{
    public LightIndividualSubsystem lightIndividualSubsystem;
    public int start;
    public int stop;
    public int hue; 
    public int saturation;

    public LoadingBarCommand(LightIndividualSubsystem lightIndividualSubsystem, int start, int stop, int hue, int saturation) {
        this.lightIndividualSubsystem = lightIndividualSubsystem;
        this.start = start;
        this.stop = stop;
        this.hue = hue;
        this.saturation = saturation;
        addRequirements(lightIndividualSubsystem);
    }

    @Override
    public void execute() {
        lightIndividualSubsystem.loadingBarColor(start, stop, hue, saturation);
    }
}
