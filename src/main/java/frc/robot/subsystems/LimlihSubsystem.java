package frc.robot.subsystems;

import org.ejml.equation.IntegerSequence.For;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.LimelightHelpers;
import frc.robot.utilities.LimelightHelpers.LimelightResults;
import frc.robot.utilities.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.utilities.LimelightHelpers.Results;

public class LimlihSubsystem extends SubsystemBase {

    // GenericEntry
    // george=Shuffleboard.getTab("ikfsdal").add("george",0).getEntry();
    double[] hrm;
    String limelightHelpNetworkTableName = "limelight-limlih";
    LimelightTarget_Fiducial[] limelightResults;
    GenericEntry x = Shuffleboard.getTab("limPose").add("x", 0).getEntry();
    GenericEntry y = Shuffleboard.getTab("limPose").add("y", 0).getEntry();
    GenericEntry rot = Shuffleboard.getTab("limPose").add("rot", 0).getEntry();
    GenericEntry z = Shuffleboard.getTab("limPose").add("z", 0).getEntry();

    public LimlihSubsystem() {

    }

    public boolean getTargetVisible(int id) {
        for (LimelightTarget_Fiducial LIMGHT : limelightResults) {
            if (LIMGHT.fiducialID == id) {
                return true;
            }
        }
        return false;
    }

    /**
     * Pulls target x (the difference as an angle with respect to x value of our
     * robot and the target)
     * from our limligh via networkTableEntries
     * 
     * @return the target x angle
     */
    public double getTargetX(int id) {

        return getFiducial(id).tx;

    }

    /**
     * Pulls target y (the difference as an angle with respect to y value of our
     * robot and the target)
     * from our limligh via networkTableEntries
     * 
     * @return the target y angle
     */
    public double getTargetY(int id) {

        return getFiducial(id).ty;
    }

    /**
     * gives how much of the camera a target covers
     * 
     * @return target area
     */
    public double getTargetA(int id) {

        return getFiducial(id).ta;
    }

    /*
     * public double getTargetId(int id) {
     * 
     * return LimelightHelpers.getFiducialID(limelightHelpNetworkTableName);
     * }
     */
    /**
     * Pose calculated with a single marker
     * 
     * @param id
     * @return Pose
     */
    public Pose2d getRobotFieldPoseByTag(int id) {
        return getFiducial(id).getRobotPose_FieldSpace2D();
    }
    
    /**
     * Pose calculated with all markers
     * 
     * @return Pose
     */
    public Pose2d getRobotPose() {
        return LimelightHelpers.getBotPose2d(limelightHelpNetworkTableName);
    }

    public Pose3d getTargetPoseInRobotSpace(int id) {
        return getFiducial(id).getTargetPose_RobotSpace();
    }

    public double getTargetId(int id) {

        return LimelightHelpers.getFiducialID(limelightHelpNetworkTableName);
    }
    
    public Pose3d getTargetSpacePose(int id) {
        return getFiducial(id).getRobotPose_TargetSpace();
    }

    public double getCalculatedPoseX(int id) {

        return getTargetSpacePose(id).getX();
    }

    public double getCalculatedPoseY(int id) {
        return getTargetSpacePose(id).getY();
    }

    public double getCalculatedPoseRot(int id) {

        return getTargetSpacePose(id).getRotation().getY();
    }

    public void switchPipeline(int pipeline) {

        LimelightHelpers.setPipelineIndex(limelightHelpNetworkTableName, pipeline);
    }

    public double getPipeline() {

        return LimelightHelpers.getCurrentPipelineIndex(limelightHelpNetworkTableName);
    }

    public LimelightTarget_Fiducial getFiducial(int id) {
        for (LimelightTarget_Fiducial LIMGHT : limelightResults) {
            if (LIMGHT.fiducialID == id) {
                return LIMGHT;
            }
        }
        return null;
    }

    public boolean seeingAnything() {
        if (limelightResults.length > 0) {
            return true;
        } else {
            return false;
        }
    }

    private void updateInputs() {
       
        boolean[] seeingThings = new boolean[16];
        for (int i = 0; i < 16; i++) {
            seeingThings[i] = getFiducial(i) != null;
        }
        Logger.recordOutput("kdsmfkl", seeingThings);
    }

    public LimelightTarget_Fiducial limelightTarget_Fiducial(int id) {

        LimelightTarget_Fiducial limelightTarget_Fiducial = new LimelightTarget_Fiducial();
        for (LimelightTarget_Fiducial LIMGHT : limelightResults) {
            if (LIMGHT.fiducialID == id) {
                return LIMGHT;
            }
        }
        return null;
    }

    @Override
    public void periodic() {

        limelightResults = LimelightHelpers
                .getLatestResults(limelightHelpNetworkTableName).targetingResults.targets_Fiducials;

        updateInputs();
        limelightResults = LimelightHelpers
                .getLatestResults(limelightHelpNetworkTableName).targetingResults.targets_Fiducials;
        if (getTargetVisible(4)) {
            Pose3d fixEverything = getTargetSpacePose(4);
            if (fixEverything != null) {
                x.setDouble(fixEverything.getX());
                y.setDouble(fixEverything.getY());
                rot.setDouble(fixEverything.getRotation().getY());
                z.setDouble(fixEverything.getZ());        
            }
        }
    }

}