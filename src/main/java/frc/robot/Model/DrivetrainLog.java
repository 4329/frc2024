package frc.robot.Model;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class DrivetrainLog {
  public SwerveModuleState[] swerveModuleStates;
  public Rotation2d rotation;
}
