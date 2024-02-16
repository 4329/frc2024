package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Model.PoseEstimationLog;
import frc.robot.Model.PoseEstimationLogAutoLogged;
import frc.robot.subsystems.swerve.Drivetrain;

public class PoseEstimationSubsystem extends SubsystemBase {

    PoseEstimationLogAutoLogged poseEstimationLogAutoLogged;

    private final Drivetrain drivetrain;
    private final VisionSubsystem visionSubsystem;
    private final SwerveDrivePoseEstimator estimator;
    private final double pathPlannerFieldWidth = 8.21;
    private final double pathPlannerFieldLength = 16.54;
    private Field2d field = new Field2d();
    private Pose2d jfdsajfks = new Pose2d();

    public PoseEstimationSubsystem(Drivetrain drivetrain, VisionSubsystem visionSubsystem) {
        this.drivetrain = drivetrain;
        this.visionSubsystem = visionSubsystem;
        poseEstimationLogAutoLogged = new PoseEstimationLogAutoLogged();
        estimator = new SwerveDrivePoseEstimator(
                Constants.DriveConstants.kDriveKinematics,
                drivetrain.getGyro(),
                drivetrain.getModulePositions(),
                drivetrain.getPose());
        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            field.setRobotPose(new Pose2d(0, 5, new Rotation2d()));
            jfdsajfks = pose != null ? pose : new Pose2d();
        });

        // Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            field.getObject("target pose").setPose(pose);
        });

        // Logging callback for the active path, this is sent as a list of poses
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            // Do whatever you want with the poses here
            field.getObject("path").setPoses(poses);
        });
        Shuffleboard.getTab("field").add("field",field);
    }

    public Pose2d getPose() {
        return estimator.getEstimatedPosition();
    }

    private void updateEstimation() {
        estimator.update(drivetrain.getGyro(), drivetrain.getModulePositions());
        if (visionSubsystem.seeingAnything()) {
            estimator.addVisionMeasurement(visionSubsystem.getRobotPose(), Timer.getFPGATimestamp());
        }
    }

    @Override
    public void periodic() {
        updateEstimation();

        updateInputs(poseEstimationLogAutoLogged);


         System.out.println("drive pose is" + getPose());

         System.out.println("lime pose is" + visionSubsystem.getRobotPose());
    }

    private void updateInputs(PoseEstimationLog poseEstimationLog) {
        poseEstimationLog.combined = transformFieldToAdvantageKit(getPose());
        poseEstimationLog.limOnly = visionSubsystem.getRobotPose();
        poseEstimationLog.driveOnly = transformFieldToAdvantageKit(drivetrain.getPose());
        poseEstimationLog.pathPlannerPosy = jfdsajfks;
        Logger.processInputs("Estimated Field Position", poseEstimationLogAutoLogged);
    }

    private Pose2d transformFieldToAdvantageKit(Pose2d pose) {
        return new Pose2d(
                pose.getX() + (Constants.FieldConstants.fieldWidth / 2),
                pose.getY() + (Constants.FieldConstants.fieldLength / 2),
                pose.getRotation());
    }

    private Pose2d transformFieldToPathPlanner(Pose2d pose) {
        return new Pose2d(            
                pose.getX() + (pathPlannerFieldLength / 2),
                pose.getY() + (pathPlannerFieldWidth / 2),

                pose.getRotation());
    }

    public Pose2d getPathPlannerStuff() {
        return transformFieldToPathPlanner(getPose());
    }

    
}
