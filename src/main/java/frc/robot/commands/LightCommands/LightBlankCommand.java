package frc.robot.commands.LightCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.lightSubsystem.LightSubsystem;

public class LightBlankCommand extends Command {
    public LightSubsystem lightIndividualSubsystem;

    public LightBlankCommand(LightSubsystem lightIndividualSubsystem) {
        this.lightIndividualSubsystem = lightIndividualSubsystem;
        addRequirements(lightIndividualSubsystem);
    }

    @Override
    public void initialize() {

        lightIndividualSubsystem.blank();
    }

}