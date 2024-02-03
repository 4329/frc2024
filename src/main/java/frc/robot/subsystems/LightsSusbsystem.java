package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightsSusbsystem extends SubsystemBase {
    Spark Strand;
    GenericEntry Light;

    public double lighte = 0.99;

    public LightsSusbsystem() {
        Strand = new Spark(1);
        //System.out.println(Strand + "hhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhh");
        Light = Shuffleboard.getTab("LED").add("Light", 0).getEntry();

    }

    public void stop() {
        Strand.set(0.99);
    }

    public void setLight(double Lights){
        Strand.set(Lights);
    }

    @Override
    public void periodic() {
        //Strand.set(Light.getDouble(0));
        // Strand.set();
    }
}
