package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LightIndividualSubsystem;

public class LightIndividualCommand extends Command {
    public LightIndividualSubsystem lightIndividualSubsystem;
    public double progress;
    public int hue; 
    public int saturation;

    public LightIndividualCommand(LightIndividualSubsystem lightIndividualSubsystem, double progress, int hue, int saturation) {
        this.lightIndividualSubsystem = lightIndividualSubsystem;
        this.progress = progress;
        this.hue = hue;
        this.saturation = saturation;
        addRequirements(lightIndividualSubsystem);
    }

    @Override
    public void execute() {

        lightIndividualSubsystem.rainbow();
        System.out.println("rainbow+++++++++++++++++++");
    }

    // @Override
    // public boolean isFinished() {
    //     return true;
    // }


}
