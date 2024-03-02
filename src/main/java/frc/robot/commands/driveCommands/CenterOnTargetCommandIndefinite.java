package frc.robot.commands.driveCommands;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swerve.Drivetrain;

public class CenterOnTargetCommandIndefinite extends CenterOnTargetCommand {
    public CenterOnTargetCommandIndefinite(VisionSubsystem visionSubsystem, Drivetrain m_drivetrain, int targetId,
            CommandXboxController xboxController) {
        super(visionSubsystem, m_drivetrain, targetId, xboxController);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
