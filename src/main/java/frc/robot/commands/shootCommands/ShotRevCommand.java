package frc.robot.commands.shootCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShootSubsystem;

public class ShotRevCommand extends Command{
    
    private ShootSubsystem shootSubsystem;
    private double setPoint = 5000; 
    

    public ShotRevCommand(ShootSubsystem shootSubsystem){
        this.shootSubsystem = shootSubsystem;
        addRequirements(shootSubsystem);
    }

    @Override
    public void initialize() {
        shootSubsystem.changeSetpoint(setPoint);
    }


    @Override
    public boolean isFinished() {

        return shootSubsystem.atSetpoint();
    }



}
