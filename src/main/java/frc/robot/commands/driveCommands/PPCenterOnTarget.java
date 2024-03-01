package frc.robot.commands.driveCommands;

import java.util.Optional;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;

public class PPCenterOnTarget extends Command {
    private VisionSubsystem visionSubsystem;
    private int tagID;

    public PPCenterOnTarget(VisionSubsystem visionSubsystem, int tagID) {
        this.visionSubsystem = visionSubsystem;
        this.tagID = tagID;
        PPHolonomicDriveController.setRotationTargetOverride(this::overrideRotation);
    }

    public Optional<Rotation2d> overrideRotation() {
        if (this.isScheduled())
            return Optional.of(new Rotation2d(visionSubsystem.faceTag(tagID)));
        return Optional.empty();
    }
}
