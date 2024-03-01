package frc.robot.subsystems;

import org.ejml.equation.IntegerSequence.For;
import org.littletonrobotics.junction.Logger;

import com.fasterxml.jackson.databind.annotation.JsonPOJOBuilder.Value;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.visionCommands.CheckLimelightCommand;
import frc.robot.utilities.AprilTagUtil;
import frc.robot.utilities.LimelightHelpers;
import frc.robot.utilities.LimelightHelpers.LimelightResults;
import frc.robot.utilities.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.utilities.LimelightHelpers.Results;

public class LimlihSubsystem extends SubsystemBase implements VisionSubsystem {

    // GenericEntry
    // george=Shuffleboard.getTab("ikfsdal").add("george",0).getEntry();
    double[] hrm;
    String limelightHelpNetworkTableName = "limelight-limlih";
    LimelightTarget_Fiducial[] limelightResults;
    private GenericEntry zGE;

    private Timer timer;
    private CheckLimelightCommand checkLimelightCommand;

    public LimlihSubsystem() {
        timer = new Timer();
        timer.start();
        checkLimelightCommand = new CheckLimelightCommand();
        
        zGE = Shuffleboard.getTab("Arm Angle").add("zPose", 0).getEntry();
    }

    public boolean CameraConnected() {
        return checkLimelightCommand.isConnected();
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

        LimelightTarget_Fiducial limetarget = getFiducial(id);
        if (limetarget != null) {

            return limetarget.getTargetPose_RobotSpace();

        }
        return null;
    }

    public Pose3d getTargetSpacePose(int id) {

        return getFiducial(id).getRobotPose_TargetSpace();
    }

    public void switchPipeline(int pipeline) {

        LimelightHelpers.setPipelineIndex(limelightHelpNetworkTableName, pipeline);
    }

    public double getPipeline() {

        return LimelightHelpers.getCurrentPipelineIndex(limelightHelpNetworkTableName);
    }

    private LimelightTarget_Fiducial getFiducial(int id) {
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
        Logger.recordOutput("Tags Seen", seeingThings);
    }

    public LimelightTarget_Fiducial limelightTarget_Fiducial(int id) {
        for (LimelightTarget_Fiducial LIMGHT : limelightResults) {
            if (LIMGHT.fiducialID == id) {
                return LIMGHT;
            }
        }
        return null;
    }

    @Override
    public void periodic() {

        if (checkLimelightCommand.isConnected()) {
            limelightResults = LimelightHelpers
            .getLatestResults(limelightHelpNetworkTableName).targetingResults.targets_Fiducials;
            Pose3d pose3d = getTargetPoseInRobotSpace(AprilTagUtil.getAprilTagSpeakerIDAprilTagIDSpeaker());
            if (pose3d != null) {

                zGE.setDouble(pose3d.getZ());

            }
        }
        
        updateInputs();
        
        occasionalCheck();
    }

    private void occasionalCheck() {
        if (timer.hasElapsed(4) && !checkLimelightCommand.isScheduled()) {
            checkLimelightCommand.schedule();
            timer.reset();
        }
    }

    @Override
    public double getTargetX(int id) {
        return getFiducial(id).tx;
    }

}