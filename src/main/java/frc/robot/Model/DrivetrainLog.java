package frc.robot.Model;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

@AutoLog
public class DrivetrainLog {
    public SwerveModuleState[] swerveModuleStates;
    public Rotation2d rotation;
}
