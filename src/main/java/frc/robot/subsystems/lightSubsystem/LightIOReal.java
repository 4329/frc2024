package frc.robot.subsystems.lightSubsystem;

import java.awt.Color;
import java.util.List;
import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightIOReal implements LightIO {

    private AddressableLED addressableLED;
    private AddressableLEDBuffer addressableLEDBuffer;


    public LightIOReal(int port, int length) {
        addressableLED = new AddressableLED(port);
        addressableLEDBuffer = new AddressableLEDBuffer(length);
        addressableLED.setLength(addressableLEDBuffer.getLength());
        addressableLED.setData(addressableLEDBuffer);
        addressableLED.start();
    }

    @Override
    public void setHSV(int index, int hue, int saturation, int lightness) {
        addressableLEDBuffer.setHSV(index, hue, saturation, lightness);
    }

    @Override
    public int getLength() {
        return addressableLEDBuffer.getLength();
    }

    @Override
    public Color8Bit getLED8Bit(int index) {
        return addressableLEDBuffer.getLED8Bit(index);
    }

    @Override
    public void periodic() {
        addressableLED.setData(addressableLEDBuffer);
    }

}
