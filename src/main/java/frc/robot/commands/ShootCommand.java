package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShootSubsystem;

public class ShootCommand extends Command{
    
    private ShootSubsystem shootSubsystem;
    private double setPoint = 1500; 
    

    public ShootCommand(ShootSubsystem shootSubsystem){
        this.shootSubsystem = shootSubsystem;
        addRequirements(shootSubsystem);
    }

    @Override
    public void initialize() {
        shootSubsystem.changeSetpoint(setPoint);
        System.out.println("initilize was called iiiiiiii");
    }

    @Override
    public void execute() {
        //shootSubsystem.shoot();
        System.out.println("execute was called eeeeeeeeeee");
    }


    @Override
    public boolean isFinished() {

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("end was called uuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuu");
        shootSubsystem.stop();
        shootSubsystem.changeSetpoint(0);
    }


}
