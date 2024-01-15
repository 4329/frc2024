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

//    GenericEntry george=Shuffleboard.getTab("ikfsdal").add("george",0).getEntry();
    double[] hrm;
    String limelightHelpNetworkTableName = "limelight-limlih";
    LimelightTarget_Fiducial[] limelightResults;

    public LimlihSubsystem() {

    }

    public boolean getTargetVisible(int id) {
       boolean targetVisible = false;
       for (LimelightTarget_Fiducial LIMGHT : limelightResults) {
            if (LIMGHT.fiducialID == id) {
                targetVisible = true;
            }
        }
        return targetVisible;
    }

    /**
     * Pulls target x (the difference as an angle with respect to x value of our
     * robot and the target)
     * from our limligh via networkTableEntries
     * 
     * @return the target x angle
     */
    public double getTargetX(int id) {

        return limelightTarget_Fiducial(id).tx;

    }

    public double getCalculatedPoseZ(int id) {

        return getPose(id).getX();
    }

    public double getCalculatedPoseRot(int id) {

        return getPose(id).getRotation().getDegrees();
    }

    /**
     * Pulls target y (the difference as an angle with respect to y value of our
     * robot and the target)
     * from our limligh via networkTableEntries
     * 
     * @return the target y angle
     */
    public double getTargetY(int id) {
        
        return limelightTarget_Fiducial(id).ty;
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

   /*  public double getTargetId(int id) {

        return LimelightHelpers.getFiducialID(limelightHelpNetworkTableName);
    }
*/
    public Pose2d getPose(int id) {
       return limelightTarget_Fiducial(id).getRobotPose_FieldSpace2D();
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
                limelightTarget_Fiducial = LIMGHT;
            }
        }
        return limelightTarget_Fiducial;
    }

    @Override
    public void periodic() {

        limelightResults = LimelightHelpers
                .getLatestResults(limelightHelpNetworkTableName).targetingResults.targets_Fiducials;

        updateInputs();
    }

}