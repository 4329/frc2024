package frc.robot.commands.driveCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swerve.Drivetrain;

public class ResetOdometryTargetSpaceCommand extends Command {

    VisionSubsystem visionSubsystem;
    Drivetrain drivetrain;
    int targetID;

    public ResetOdometryTargetSpaceCommand(VisionSubsystem visionSubsystem, Drivetrain drivetrain, int targetID) {

        this.visionSubsystem = visionSubsystem;
        this.drivetrain = drivetrain;
        this.targetID = targetID;
    }

    @Override
    public void initialize() {
        if (visionSubsystem.CameraConnected() && visionSubsystem.getTargetVisible(targetID))
            drivetrain.resetOdometry(visionSubsystem.getTargetSpacePose(targetID).toPose2d());
    }

    @Override
    public boolean isFinished() {

        return true;
    }
}
