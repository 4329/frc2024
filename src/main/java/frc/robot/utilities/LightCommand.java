package frc.robot.utilities;

import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.subsystems.lightSubsystem.LightSubsystem;
import java.util.Set;
import java.util.function.BooleanSupplier;

public class LightCommand extends Command {
  private LightCommandScheduler lightCommandScheduler;

  public LightCommand(LightSubsystem lightSubsystem) {
    this.lightCommandScheduler = lightSubsystem.lightCommandScheduler;
  }

  @Override
  public ParallelCommandGroup alongWith(Command... parallel) {
    // TODO Auto-generated method stub
    return super.alongWith(parallel);
  }

  @Override
  public SequentialCommandGroup andThen(Command... next) {
    // TODO Auto-generated method stub
    return super.andThen(next);
  }

  @Override
  public SequentialCommandGroup andThen(Runnable toRun, Subsystem... requirements) {
    // TODO Auto-generated method stub
    return super.andThen(toRun, requirements);
  }

  @Override
  public ProxyCommand asProxy() {
    // TODO Auto-generated method stub
    return super.asProxy();
  }

  @Override
  public SequentialCommandGroup beforeStarting(Command before) {
    // TODO Auto-generated method stub
    return super.beforeStarting(before);
  }

  @Override
  public SequentialCommandGroup beforeStarting(Runnable toRun, Subsystem... requirements) {
    // TODO Auto-generated method stub
    return super.beforeStarting(toRun, requirements);
  }

  @Override
  public ParallelDeadlineGroup deadlineWith(Command... parallel) {
    // TODO Auto-generated method stub
    return super.deadlineWith(parallel);
  }

  @Override
  public void end(boolean interrupted) {
    // TODO Auto-generated method stub
    super.end(interrupted);
  }

  @Override
  public WrapperCommand finallyDo(BooleanConsumer end) {
    // TODO Auto-generated method stub
    return super.finallyDo(end);
  }

  @Override
  public WrapperCommand finallyDo(Runnable end) {
    // TODO Auto-generated method stub
    return super.finallyDo(end);
  }

  @Override
  public InterruptionBehavior getInterruptionBehavior() {
    // TODO Auto-generated method stub
    return super.getInterruptionBehavior();
  }

  @Override
  public String getName() {
    // TODO Auto-generated method stub
    return super.getName();
  }

  @Override
  public Set<Subsystem> getRequirements() {
    // TODO Auto-generated method stub
    return super.getRequirements();
  }

  @Override
  public String getSubsystem() {
    // TODO Auto-generated method stub
    return super.getSubsystem();
  }

  @Override
  public WrapperCommand handleInterrupt(Runnable handler) {
    // TODO Auto-generated method stub
    return super.handleInterrupt(handler);
  }

  @Override
  public boolean hasRequirement(Subsystem requirement) {
    // TODO Auto-generated method stub
    return super.hasRequirement(requirement);
  }

  @Override
  public WrapperCommand ignoringDisable(boolean doesRunWhenDisabled) {
    // TODO Auto-generated method stub
    return super.ignoringDisable(doesRunWhenDisabled);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Command");
    builder.addStringProperty(".name", this::getName, null);
  }

  @Override
  public boolean isFinished() {
    // TODO Auto-generated method stub
    return super.isFinished();
  }

  @Override
  public ConditionalCommand onlyIf(BooleanSupplier condition) {
    // TODO Auto-generated method stub
    return super.onlyIf(condition);
  }

  @Override
  public ParallelRaceGroup onlyWhile(BooleanSupplier condition) {
    // TODO Auto-generated method stub
    return super.onlyWhile(condition);
  }

  @Override
  public ParallelRaceGroup raceWith(Command... parallel) {
    // TODO Auto-generated method stub
    return super.raceWith(parallel);
  }

  @Override
  public RepeatCommand repeatedly() {
    // TODO Auto-generated method stub
    return super.repeatedly();
  }

  @Override
  public boolean runsWhenDisabled() {
    // TODO Auto-generated method stub
    return super.runsWhenDisabled();
  }

  @Override
  public void setName(String name) {
    // TODO Auto-generated method stub
    super.setName(name);
  }

  @Override
  public void setSubsystem(String subsystem) {
    // TODO Auto-generated method stub
    super.setSubsystem(subsystem);
  }

  @Override
  public ConditionalCommand unless(BooleanSupplier condition) {
    // TODO Auto-generated method stub
    return super.unless(condition);
  }

  @Override
  public ParallelRaceGroup until(BooleanSupplier condition) {
    // TODO Auto-generated method stub
    return super.until(condition);
  }

  @Override
  public WrapperCommand withInterruptBehavior(InterruptionBehavior interruptBehavior) {
    // TODO Auto-generated method stub
    return super.withInterruptBehavior(interruptBehavior);
  }

  @Override
  public WrapperCommand withName(String name) {
    // TODO Auto-generated method stub
    return super.withName(name);
  }

  @Override
  public ParallelRaceGroup withTimeout(double seconds) {
    // TODO Auto-generated method stub
    return super.withTimeout(seconds);
  }

  @Override
  public void initialize() {
    if (MathUtils.getCallerClassName() != LightCommandScheduler.class.getName()) {
      schedule();
      return;
    }
  }

  @Override
  public void execute() {
    if (MathUtils.getCallerClassName() != LightCommandScheduler.class.getName()) {
      System.out.println(
          MathUtils.getCallerClassName()
              + "[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]");
      return;
    }
  }

  @Override
  public void schedule() {
    lightCommandScheduler.scheduleCommand(this);
  }

  @Override
  public void cancel() {
    lightCommandScheduler.cancelCommand(this);
  }

  @Override
  public boolean isScheduled() {
    return lightCommandScheduler.isScheduled(this);
  }
}
