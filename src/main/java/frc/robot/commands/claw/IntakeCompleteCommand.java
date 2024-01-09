package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ColorDetectorSubsystem;

public class IntakeCompleteCommand extends Command {

    private ClawSubsystem clawSubsystem;
    private ColorDetectorSubsystem colorDetector;

    public IntakeCompleteCommand(ClawSubsystem clawSubsystem, ColorDetectorSubsystem colorDetector) {

        this.clawSubsystem = clawSubsystem;
        this.colorDetector = colorDetector;
        addRequirements(clawSubsystem);

    }

    @Override
    public void initialize() {

        clawSubsystem.intake();

    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {

        return colorDetector.holdingSomthing();

    }

    @Override
    public void end(boolean interrupted) {

        clawSubsystem.stop();
    
    }


}
