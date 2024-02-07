package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Model.LineBreakAutoLogged;

public class LineBreakSensorSubsystem extends SubsystemBase{

    private DigitalInput lineBreak;
    private LineBreakAutoLogged lineBreakAutoLogged;

    public LineBreakSensorSubsystem() {

        this.lineBreak = new DigitalInput(0);
        lineBreakAutoLogged = new LineBreakAutoLogged();
    }

    @Override
    public void periodic() {
         
        lineBreakAutoLogged.notBroken = isNotBroken();
        Logger.processInputs("Line Break", lineBreakAutoLogged);

    }

public Boolean isNotBroken(){

    return lineBreak.get();

    }

}