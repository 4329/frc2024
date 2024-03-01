package frc.robot.subsystems;

import java.util.Map;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Model.LimlihLogAutoLogged;
import frc.robot.commands.visionCommands.CheckLimelightCommand;
import frc.robot.utilities.AprilTagUtil;
import frc.robot.utilities.LimelightHelpers;
import frc.robot.utilities.MathUtils;
import frc.robot.utilities.LimelightHelpers.LimelightTarget_Fiducial;

public class LimlihSubsystem extends SubsystemBase implements VisionSubsystem {

    // GenericEntry
    // george=Shuffleboard.getTab("ikfsdal").add("george",0).getEntry();
    double[] hrm;
    String limelightHelpNetworkTableName = "limelight-limlih";
    LimelightTarget_Fiducial[] limelightResults;
    private GenericEntry zGE;
    private GenericEntry sight;

    private Timer timer;
    private CheckLimelightCommand checkLimelightCommand;

    private LimlihLogAutoLogged limlihLogAutoLogged;

    public LimlihSubsystem(CheckLimelightCommand checkLimelightCommand) {
        timer = new Timer();
        timer.start();
        this.checkLimelightCommand = checkLimelightCommand;

        zGE = Shuffleboard.getTab("shoot").add("zPose", 0).getEntry();
        sight = Shuffleboard.getTab("RobotData").add("Seeing Speaker", false).withPosition(3, 0).withSize(2, 2)
                .withProperties(Map.of("Color when true", "#FFFFFF", "Color when false", "#000000")).getEntry();

        limlihLogAutoLogged = new LimlihLogAutoLogged();
    }

    public boolean CameraConnected() {
        return checkLimelightCommand.isConnected();
    }

    public boolean getTargetVisible(int id) {
        if (limelightResults == null) {
            return false;
        }
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
        if (limelightResults == null)
            return null;
        for (LimelightTarget_Fiducial LIMGHT : limelightResults) {
            if (LIMGHT.fiducialID == id) {
                return LIMGHT;
            }
        }
        return null;
    }

    public boolean seeingAnything() {
        if (limelightResults != null && limelightResults.length > 0) {
            return true;
        } else {
            return false;
        }
    }

    private void updateInputs() {

        for (int i = 0; i < 16; i++) {
            limlihLogAutoLogged.tvs[i] = getTargetVisible(i);
            if (limlihLogAutoLogged.tvs[i]) {
                limlihLogAutoLogged.tXs[i] = getTargetX(i);
                limlihLogAutoLogged.tagPoses[i] = MathUtils.addPoses3D(getTargetPoseInRobotSpace(i), MathUtils.pose2DtoPose3D(getRobotPose()));
            }else {
                limlihLogAutoLogged.tXs[i] = 0;
                limlihLogAutoLogged.tagPoses[i] = new Pose3d();
            }
        }
        limlihLogAutoLogged.limlihconnected = CameraConnected();

        Logger.processInputs("Limlihsubsystem", limlihLogAutoLogged);
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

        sight.setBoolean(getTargetVisible(AprilTagUtil.getAprilTagSpeakerIDAprilTagIDSpeaker()));

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

    @Override
    public double faceTag(int id) {
        Pose2d initialPose = getTargetSpacePose(id).toPose2d();
        return Math.atan2(initialPose.getX(), initialPose.getY());
    }

}