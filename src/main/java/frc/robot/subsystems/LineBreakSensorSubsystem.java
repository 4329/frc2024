package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Model.LineBreakAutoLogged;
import frc.robot.subsystems.LoggingSubsystem.LoggedSubsystem;

public class LineBreakSensorSubsystem extends SubsystemBase implements LoggedSubsystem {

    private DigitalInput lineBreak;
    private LineBreakAutoLogged lineBreakAutoLogged;

    public LineBreakSensorSubsystem() {

        this.lineBreak = new DigitalInput(0);
        lineBreakAutoLogged = new LineBreakAutoLogged();
    }

    @Override
    public LoggableInputs log() {
         
        lineBreakAutoLogged.notBroken = isNotBroken();
        return lineBreakAutoLogged;
    }

public Boolean isNotBroken(){

    return lineBreak.get();

    }

}