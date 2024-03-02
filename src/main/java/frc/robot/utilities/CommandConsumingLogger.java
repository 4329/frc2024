package frc.robot.utilities;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Model.CommandLogEntryAutoLogged;
import org.littletonrobotics.junction.Logger;

import java.util.function.Consumer;

public class CommandConsumingLogger implements Consumer<Command> {

    private final String message;
    private CommandLogEntryAutoLogged commandLogEntryAutoLogged = new CommandLogEntryAutoLogged();

    public CommandConsumingLogger(String message) {
        this.message = message;
    }
    @Override
    public void accept(Command command) {
        commandLogEntryAutoLogged.command = command.getName();
        commandLogEntryAutoLogged.message = message;
        System.out.println("COMMAND: " + command.getName() + " " + message);
        Logger.processInputs("Commands", commandLogEntryAutoLogged);
    }
}
