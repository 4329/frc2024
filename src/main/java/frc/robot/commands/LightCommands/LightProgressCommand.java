package frc.robot.commands.LightCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.lightSubsystem.LightSubsystem;

public class LightProgressCommand extends Command {
    public LightSubsystem lightIndividualSubsystem;
    public double progress;
    public int hue; 
    public int saturation;

    public LightProgressCommand(LightSubsystem lightIndividualSubsystem, double progress, int hue, int saturation) {
        this.lightIndividualSubsystem = lightIndividualSubsystem;
        this.progress = progress;
        this.hue = hue;
        this.saturation = saturation;
        addRequirements(lightIndividualSubsystem);
    }

    @Override
    public void execute() {

        lightIndividualSubsystem.progressBarColorrrr(progress, hue, saturation);
        System.out.println("rainbow+++++++++++++++++++");
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        lightIndividualSubsystem.beforeMatchColors();
    }


}
