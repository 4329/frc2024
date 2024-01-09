package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ColorDetectorSubsystem.FieldElement;

public class SimpleOuttakeCommand extends Command {

    private ClawSubsystem clawSubsystem;

    public SimpleOuttakeCommand(ClawSubsystem clawSubsystem) {

        this.clawSubsystem = clawSubsystem;
        addRequirements(clawSubsystem);

    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

        clawSubsystem.outtakeLow(FieldElement.CONE);

    }

    @Override
   public boolean isFinished() {

            return false;
            
    }

    @Override
    public void end(boolean interrupted) {

        clawSubsystem.stop();

    }

}