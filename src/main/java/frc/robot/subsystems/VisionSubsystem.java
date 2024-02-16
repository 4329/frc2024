package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.utilities.LimelightHelpers.LimelightTarget_Fiducial;

public interface VisionSubsystem {

    boolean CameraConnected();

    boolean getTargetVisible(int id);

    double getTargetX(int id);

    /**
     * Pose calculated with a single marker
     * 
     * @param id
     * @return Pose
     */
    Pose2d getRobotFieldPoseByTag(int id);

    /**
     * Pose calculated with all markers
     * 
     * @return Pose
     */
    Pose2d getRobotPose();

    Pose3d getTargetPoseInRobotSpace(int id);

    public Pose3d getTargetSpacePose(int id);

    void switchPipeline(int pipeline);

    boolean seeingAnything();

    LimelightTarget_Fiducial limelightTarget_Fiducial(int id);

}