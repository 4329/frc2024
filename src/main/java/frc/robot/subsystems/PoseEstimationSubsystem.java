package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Model.PoseEstimationLog;
import frc.robot.Model.PoseEstimationLogAutoLogged;
import frc.robot.subsystems.swerve.Drivetrain;

public class PoseEstimationSubsystem extends SubsystemBase {

    PoseEstimationLogAutoLogged poseEstimationLogAutoLogged;

    private final Drivetrain drivetrain;
    private final LimlihSubsystem limlihSubsystem;
    private final SwerveDrivePoseEstimator estimator;

    public PoseEstimationSubsystem(Drivetrain drivetrain, LimlihSubsystem limlihSubsystem) {
        this.drivetrain = drivetrain;
        this.limlihSubsystem = limlihSubsystem;
        poseEstimationLogAutoLogged = new PoseEstimationLogAutoLogged();
        estimator = new SwerveDrivePoseEstimator(
                Constants.DriveConstants.kDriveKinematics,
                drivetrain.getGyro(),
                drivetrain.getModulePositions(),
                drivetrain.getPose()
        );
    }

    public Pose2d getPose() {
        return estimator.getEstimatedPosition();
    }

    private void updateEstimation() {
        estimator.update(drivetrain.getGyro(), drivetrain.getModulePositions());
        if (limlihSubsystem.seeingAnything()) {
            estimator.addVisionMeasurement(limlihSubsystem.getRobotPose(), Timer.getFPGATimestamp());
        }
    }

    @Override
    public void periodic() {
        updateEstimation();

        updateInputs(poseEstimationLogAutoLogged);
    }

    private void updateInputs(PoseEstimationLog poseEstimationLog) {
        poseEstimationLog.combined = transformFieldToAdvantageKit(getPose());
        poseEstimationLog.limOnly = transformFieldToAdvantageKit(limlihSubsystem.getRobotPose());
        poseEstimationLog.driveOnly = transformFieldToAdvantageKit(drivetrain.getPose());
        Logger.processInputs("Estimated Field Position", poseEstimationLogAutoLogged);
    }

    private Pose2d transformFieldToAdvantageKit(Pose2d pose) {
        return new Pose2d(
            pose.getX() + Constants.FieldConstants.fieldWidth / 2,
            pose.getY() + Constants.FieldConstants.fieldLength / 2,
            pose.getRotation()
        );
    }
}
