package frc.robot.commands.drive;

import java.lang.annotation.Target;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimlihSubsystem;
import frc.robot.subsystems.swerve.Drivetrain;

public class ResetOdometryTargetSpaceCommand extends Command {

    LimlihSubsystem limlihSubsystem;
    Drivetrain drivetrain;
    int targetID;

    public ResetOdometryTargetSpaceCommand(LimlihSubsystem limlihSubsystem, Drivetrain drivetrain, int targetID) {

        this.limlihSubsystem = limlihSubsystem;
        this.drivetrain = drivetrain;
        this.targetID = targetID;
    }

    @Override
    public void initialize() {


        
        if (limlihSubsystem.getTargetVisible(targetID))
        drivetrain.resetOdometry(limlihSubsystem.getFiducial(targetID).getRobotPose_TargetSpace().toPose2d());
        
    }

    @Override
    public boolean isFinished() {

        return limlihSubsystem.getTargetVisible(targetID);

    }
}
