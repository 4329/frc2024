package frc.robot.commands.shootCommands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.armCommands.ArmAngleCommand;
import frc.robot.commands.indexCommands.IndexReverseForShotCommand;
import frc.robot.commands.indexCommands.OutdexSensorCommand;
import frc.robot.subsystems.ArmAngleSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.LineBreakSensorSubsystem;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.utilities.ArmAngle;
import frc.robot.utilities.UnInstantCommand;
import us.hebi.quickbuf.UninitializedMessageException;

public class ShooterSourceCommand extends SequentialCommandGroup {

    public ShooterSourceCommand(IndexSubsystem indexSubsystem, ShootSubsystem shootSubsystem,
            LineBreakSensorSubsystem lineBreakSensorSubsystem, ArmAngleSubsystem armAngleSubsystem) {

        super(

                // new ArmAngleCommand(armAngleSubsystem, ArmAngle.SHOOTERSOURCE),
                
                new ParallelRaceGroup(
                        new UnInstantCommand(() -> shootSubsystem.changeSetpoint(-1500)),
                        new OutdexSensorCommand(lineBreakSensorSubsystem, indexSubsystem)),

                
                new ParallelRaceGroup(
                        new UnInstantCommand(() -> shootSubsystem.changeSetpoint(-500)),
 
                        new IndexReverseForShotCommand(lineBreakSensorSubsystem, indexSubsystem)),
                        
                new UnInstantCommand(() -> shootSubsystem.changeSetpoint(0)));
    }

}
