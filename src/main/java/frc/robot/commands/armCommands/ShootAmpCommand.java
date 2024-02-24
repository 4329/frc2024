package frc.robot.commands.armCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.ShootSubsystem;

public class ShootAmpCommand extends Command {
    private ShootSubsystem shootSubsystem;
    private IndexSubsystem indexSubsystem;
    private double setPoint = 1500;
    private Timer timer = new Timer();

    public ShootAmpCommand(ShootSubsystem shootSubsystem, IndexSubsystem indexSubsystem) {
        this.shootSubsystem = shootSubsystem;
        this.indexSubsystem = indexSubsystem;
        addRequirements(shootSubsystem, indexSubsystem);
    }

    @Override
    public void initialize() {
        shootSubsystem.changeSetpoint(setPoint);
    }

    @Override
    public void execute() {
        if (shootSubsystem.atSetpoint()) {
            indexSubsystem.in();
            timer.start();
        }
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(0.5);
    }

    @Override
    public void end(boolean interrupted) {
        shootSubsystem.stop();
        indexSubsystem.stop();
    }

}
