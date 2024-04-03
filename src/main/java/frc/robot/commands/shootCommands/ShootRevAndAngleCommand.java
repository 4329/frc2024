package frc.robot.commands.shootCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ArmAngleSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.utilities.ElevatorSetpoints;

public class ShootRevAndAngleCommand extends ParallelCommandGroup {

    public ShootRevAndAngleCommand(ArmAngleSubsystem armAngleSubsystem, ShootSubsystem shootSubsystem, VisionSubsystem visionSubsystem, ElevatorSubsystem elevatorSubsystem) {

        super(
            new ShooterAimCommandIndefinite(visionSubsystem, armAngleSubsystem, elevatorSubsystem)); 
            new ShooterShotCommandIndefinite(shootSubsystem, visionSubsystem);
            
    }
    
}
