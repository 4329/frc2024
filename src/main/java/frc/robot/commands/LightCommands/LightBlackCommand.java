package frc.robot.commands.LightCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LightIndividualSubsystem;

public class LightBlackCommand extends Command{
   public LightIndividualSubsystem lightIndividualSubsystem;

    public LightBlackCommand(LightIndividualSubsystem lightIndividualSubsystem){
        this.lightIndividualSubsystem = lightIndividualSubsystem;
        addRequirements(lightIndividualSubsystem);
    }
    
    @Override
    public void initialize() {
        System.out.println("d____________________________________________________________________________________________________________________jidljldjkjlkjjkjdlkjkldjlkjdjdkljd");
        lightIndividualSubsystem.setBlack();
    }
    
    @Override
    public void execute() {
    }
    
    @Override
    public boolean isFinished() {
       return false;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("sdfaijlfidjsalifjsdlijfsadlifjdsilifjodsjafjiodj");
    }

}
