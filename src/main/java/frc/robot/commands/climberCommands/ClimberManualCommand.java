package frc.robot.commands.climberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmAngleSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.LightSubsystem;
import frc.robot.subsystems.LightSubsystem.LEDPattern;
import frc.robot.utilities.ArmAngle;
import java.util.function.Supplier;

public class ClimberManualCommand extends Command {

  private ClimberSubsystem climberSubsystem;
  private ArmAngleSubsystem armAngleSubsystem;
  private LightSubsystem lightSubsystem;
  private Supplier<Double> rightTriggerSupplier;
  private Supplier<Double> leftTriggerSupplier;

  public ClimberManualCommand(
      ClimberSubsystem climberSubsystem,
      ArmAngleSubsystem armAngleSubsystem,
      LightSubsystem lightSubsystem,
      Supplier<Double> leftTriggerSupplier,
      Supplier<Double> rightTriggerSupplier) {

    this.climberSubsystem = climberSubsystem;
    this.armAngleSubsystem = armAngleSubsystem;
    this.lightSubsystem = lightSubsystem;

    this.leftTriggerSupplier = leftTriggerSupplier;
    this.rightTriggerSupplier = rightTriggerSupplier;
  }

  @Override
  public void initialize() {
    lightSubsystem.setLEDPattern(LEDPattern.CLIMB);
  }

  @Override
  public void execute() {
    if (rightTriggerSupplier.get() > 0.1) {
      climberSubsystem.climberMove(rightTriggerSupplier.get() * 2);

    } else if (leftTriggerSupplier.get() > 0.1) {
      climberSubsystem.climberMove(-leftTriggerSupplier.get() * 2);
      armAngleSubsystem.setArmAngle(ArmAngle.CLIMB);

    } else {

      climberSubsystem.stop();
    }
  }

  @Override
  public boolean isFinished() {

    return false;
  }

  @Override
  public void end(boolean interrupted) {}
}
