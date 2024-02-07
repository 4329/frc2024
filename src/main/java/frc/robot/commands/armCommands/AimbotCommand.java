package frc.robot.commands.armCommands;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.drive.DriveByController;
import frc.robot.commands.drive.DriveToTargetCommand;
import frc.robot.subsystems.ArmAngleSubsystem;
import frc.robot.subsystems.LimlihSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.subsystems.swerve.Drivetrain;

public class AimbotCommand extends Command {
    
    public Command aimbotCommand(LimlihSubsystem limlihSubsystem, ArmAngleSubsystem armAngleSubsystem, PoseEstimationSubsystem poseEstimationSubsystem, 
            ShootSubsystem shootSubsystem, DriveByController driveByController, CommandXboxController driverController, Drivetrain drivetrain) {
                Optional<Alliance> alliance = DriverStation.getAlliance();
                int targetId = alliance.isPresent() && alliance.get().equals(DriverStation.Alliance.Red) ? 4 : 7;
                return new SequentialCommandGroup(
                new DriveToTargetCommand(drivetrain, limlihSubsystem, targetId, -3),
                new ShooterAimCommand(limlihSubsystem, armAngleSubsystem).withTimeout(1), 
                new ShootCommand(shootSubsystem)
        );
    }

}
