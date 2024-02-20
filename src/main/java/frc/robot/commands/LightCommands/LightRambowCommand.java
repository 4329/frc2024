package frc.robot.commands.LightCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LightIndividualSubsystem;

public class LightRambowCommand extends Command{
        public LightIndividualSubsystem lightIndividualSubsystem;

    public LightRambowCommand(LightIndividualSubsystem lightIndividualSubsystem) {
        this.lightIndividualSubsystem = lightIndividualSubsystem;

        addRequirements(lightIndividualSubsystem);
    }

    @Override
    public void execute() {
        lightIndividualSubsystem.rainbow();
        System.out.println("rainbow+++++++++++++++++++");
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        lightIndividualSubsystem.beforeMatchColors();
    }
}
