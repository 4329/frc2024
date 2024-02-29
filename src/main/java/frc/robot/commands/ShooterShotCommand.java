package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.ShootSubsystem;

public class ShooterShotCommand extends Command {

    private ShootSubsystem shootSubsystem;
    private IndexSubsystem indexSubsystem;
    private double setpoint = 5000;
    private Timer timer = new Timer();

    public ShooterShotCommand(ShootSubsystem shootSubsystem, IndexSubsystem indexSubsystem) {
        this.shootSubsystem = shootSubsystem;
        this.indexSubsystem = indexSubsystem;
        addRequirements(shootSubsystem);

    }

    @Override
    public void initialize() {

        shootSubsystem.changeSetpoint(setpoint);
        timer.reset();
        

    }

    @Override
    public void execute() {

        if (shootSubsystem.atSetpoint() == true) {

            timer.start();
            indexSubsystem.inForShot();


        }

    }

    @Override
    public boolean isFinished() {


        return timer.hasElapsed(0.4);
    }

    @Override
    public void end(boolean interrupted) {

        shootSubsystem.stop();
        indexSubsystem.stop();
        timer.stop();

    }

}
