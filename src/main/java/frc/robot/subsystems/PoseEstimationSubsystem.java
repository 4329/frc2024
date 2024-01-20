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

    PoseEstimationLogAutoLogged a;

    private final Drivetrain drivetrain;
    private final LimlihSubsystem limlihSubsystem;
    private final SwerveDrivePoseEstimator estimator;

    public PoseEstimationSubsystem(Drivetrain drivetrain, LimlihSubsystem limlihSubsystem) {
        this.drivetrain = drivetrain;
        this.limlihSubsystem = limlihSubsystem;
        a = new PoseEstimationLogAutoLogged();
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
            estimator.addVisionMeasurement(limlihSubsystem.getPose(), Timer.getFPGATimestamp());
        }
    }

    @Override
    public void periodic() {
        updateEstimation();

        updateInputs(a);
    }

    private void updateInputs(PoseEstimationLog stupid) {
        stupid.combined = transformFieldToAdvantageKit(getPose());
        stupid.limOnly = transformFieldToAdvantageKit(limlihSubsystem.getPose());
        stupid.driveOnly = transformFieldToAdvantageKit(drivetrain.getPose());
        Logger.processInputs("ohno", a);
    }

    private Pose2d transformFieldToAdvantageKit(Pose2d pose) {
        return new Pose2d(pose.getX() + 8, pose.getY() + 4, pose.getRotation());
    }
}
