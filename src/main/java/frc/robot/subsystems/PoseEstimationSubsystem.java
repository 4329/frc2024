package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
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
    private Pose2d pathPlannerPose = new Pose2d();
    private final ArmAngleSubsystem armAngleSubsystem;

    private final double shootDexerZ = 0.484;
    private final double shootDexerX = -0.115;
    private final double shooterYawOffset = -0.1;

    public PoseEstimationSubsystem(Drivetrain drivetrain, VisionSubsystem visionSubsystem, ArmAngleSubsystem armAngleSubsystem) {
        this.visionSubsystem = visionSubsystem;
        this.drivetrain = drivetrain;
        this.armAngleSubsystem = armAngleSubsystem;

        poseEstimationLogAutoLogged = new PoseEstimationLogAutoLogged();
        estimator = new SwerveDrivePoseEstimator(
                Constants.DriveConstants.kDriveKinematics,
                drivetrain.getGyro(),
                drivetrain.getModulePositions(),
                drivetrain.getPose());

        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            field.setRobotPose(new Pose2d(0, 5, new Rotation2d()));
            pathPlannerPose = pose != null ? pose : new Pose2d();
        });

        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            field.getObject("target pose").setPose(pose);
        });

        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            field.getObject("path").setPoses(poses);
        });
        Shuffleboard.getTab("field").add("field", field);
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
    }

    private void updateInputs(PoseEstimationLog poseEstimationLog) {
        poseEstimationLog.combined = transformFieldToAdvantageKit(getPose());
        poseEstimationLog.limOnly = transformFieldToAdvantageKit(visionSubsystem.getRobotPose());
        poseEstimationLog.driveOnly = transformFieldToAdvantageKit(drivetrain.getPose());
        poseEstimationLog.pathPlannerPosy = pathPlannerPose;
        Logger.processInputs("Estimated Field Position", poseEstimationLogAutoLogged);
        Logger.recordOutput("zero", new Pose2d());
        Logger.recordOutput("zeroes", new Pose3d[] {
            new Pose3d(), // Bumper
            new Pose3d(), // Intake
            new Pose3d(), // Gearbox
            new Pose3d(shootDexerX, 0, shootDexerZ, new Rotation3d(0, (2 * Math.PI) - armAngleSubsystem.getAngleRadians() + shooterYawOffset, 0)), // Shootdexer
            new Pose3d(), // Elevator Base
            new Pose3d(0, 0, 0, new Rotation3d()) // Elevator left
        });
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
