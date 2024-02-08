package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LightIndividualSubsystem;

public class LightBlankCommand extends Command {
    public LightIndividualSubsystem lightIndividualSubsystem;

    public LightBlankCommand(LightIndividualSubsystem lightIndividualSubsystem) {
        this.lightIndividualSubsystem = lightIndividualSubsystem;
        addRequirements(lightIndividualSubsystem);
    }

    @Override
    public void initialize() {

        lightIndividualSubsystem.blank();
    }

}