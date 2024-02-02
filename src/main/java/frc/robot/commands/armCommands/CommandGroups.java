package frc.robot.commands.armCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.drive.CenterOnTargetCommand;
import frc.robot.subsystems.ArmAngleSubsystem;
import frc.robot.subsystems.LimlihSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.ShootSubsystem;

public class CommandGroups extends Command {
    
    public Command aimbotCommand(LimlihSubsystem limlihSubsystem, ArmAngleSubsystem armAngleSubsystem, ShootSubsystem shootSubsystem) {
        return new SequentialCommandGroup(new CenterOnTargetCommand(limlihSubsystem, null, 0, null).withTimeout(1), new ShooterAimCommand(null, null).withTimeout(1), new ShootCommand(null));
    }
    
}
