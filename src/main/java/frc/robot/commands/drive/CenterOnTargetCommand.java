package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.LimlihSubsystem;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.utilities.MathUtils;

public class CenterOnTargetCommand extends Command {

    private final LimlihSubsystem limlihSubsystem;
    private final Drivetrain drivetrain;
    private final int targetId;

    private final PIDController rotationPID;
    private CommandXboxController xboxController;
    private Timer timer = new Timer();


    public CenterOnTargetCommand(LimlihSubsystem limlihSubsystem, Drivetrain m_drivetrain, int targetId,
            CommandXboxController xboxController) {

        this.limlihSubsystem = limlihSubsystem;
        this.drivetrain = m_drivetrain;
        this.targetId = targetId;
        this.xboxController = xboxController;

        rotationPID = new PIDController(0.00002, 0, 0);
        addRequirements(limlihSubsystem, m_drivetrain);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();

        rotationPID.setP(0.035);
        rotationPID.setTolerance(0.2);
        rotationPID.setSetpoint(0);
    }

    @Override
    public void execute() {
        double rotationCalc = 0;
        if (limlihSubsystem.getTargetVisible(targetId)) {

            rotationCalc = rotationPID.calculate(limlihSubsystem.getTargetX(targetId));

            if (rotationCalc > Constants.DriveConstants.kMaxAngularSpeed)
                rotationCalc = Constants.DriveConstants.kMaxAngularSpeed;
            else if (rotationCalc < -Constants.DriveConstants.kMaxAngularSpeed)
                rotationCalc = -Constants.DriveConstants.kMaxAngularSpeed;
            else if (rotationPID.atSetpoint())
                rotationCalc = 0;

            double adjTranslation = ((Constants.DriveConstants.kMaxAngularSpeed - Math.abs(rotationCalc))
                    / Constants.DriveConstants.kMaxAngularSpeed) * 0.5;

            drivetrain.drive(
                    -inputTransform(xboxController.getLeftY())
                            * (Constants.DriveConstants.kMaxSpeedMetersPerSecond * adjTranslation),
                    -inputTransform(xboxController.getLeftX())
                            * (Constants.DriveConstants.kMaxSpeedMetersPerSecond * adjTranslation),
                    rotationCalc,
                    true);
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
