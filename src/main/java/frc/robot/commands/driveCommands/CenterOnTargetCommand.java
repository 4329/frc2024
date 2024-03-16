package frc.robot.commands.driveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.utilities.AprilTagUtil;
import frc.robot.utilities.MathUtils;

public class CenterOnTargetCommand extends Command {
  private final VisionSubsystem visionSubsystem;
  private final Drivetrain drivetrain;
  private int targetId;

  private final PIDController rotationPID;
  private final CommandXboxController xboxController;

  public CenterOnTargetCommand(
      VisionSubsystem visionSubsystem,
      Drivetrain m_drivetrain,
      int targetId,
      CommandXboxController xboxController) {
    this.visionSubsystem = visionSubsystem;
    this.drivetrain = m_drivetrain;
    this.targetId = targetId;
    this.xboxController = xboxController;

    rotationPID = new PIDController(0.03975, 0, 0); // 0.75, 0, 0
    rotationPID.setTolerance(0.1);
    rotationPID.setSetpoint(0);

    // rotationPID.enableContinuousInput(0, 2 * Math.PI);

    addRequirements(visionSubsystem, m_drivetrain);
  }

  @Override
  public void initialize() {

    drivetrain.drive(0, 0, 0, true);
    targetId = AprilTagUtil.getAprilTagSpeakerIDAprilTagIDSpeaker();
  }

  @Override
  public void execute() {
    double rotationCalc = 0;
    System.out.println(
        visionSubsystem.CameraConnected() + ", " + visionSubsystem.getTargetVisible(targetId));
    System.out.println(targetId);
    if (visionSubsystem.CameraConnected() && visionSubsystem.getTargetVisible(targetId)) {

      rotationCalc = rotationPID.calculate(visionSubsystem.getTargetX(targetId));

      if (rotationCalc > Constants.DriveConstants.kMaxAngularSpeed) {
        rotationCalc = Constants.DriveConstants.kMaxAngularSpeed;
      } else if (rotationCalc < -Constants.DriveConstants.kMaxAngularSpeed) {
        rotationCalc = -Constants.DriveConstants.kMaxAngularSpeed;
      } else if (rotationPID.atSetpoint()) {
        rotationCalc = 0;
      }

      double adjTranslation =
          ((Constants.DriveConstants.kMaxAngularSpeed - Math.abs(rotationCalc))
                  / Constants.DriveConstants.kMaxAngularSpeed)
              * 0.5;

      drivetrain.drive(0, 0, rotationCalc, true);
      System.out.println("EXECUTE PHOTON VISION IS DONEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE");
      System.out.println("rotation output is --> " + rotationCalc);
    }
  }

  @Override
  public boolean isFinished() {
    return rotationPID.atSetpoint();
  }

  private double inputTransform(double input) {
    return MathUtils.singedSquare(MathUtils.applyDeadband(input));
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }
}
