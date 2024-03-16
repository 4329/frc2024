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
import frc.robot.utilities.LEDAllocator.RealAllocator;

public class LightIOReal implements LightIO {

    private int length;
    private int startDex;

    private RealAllocator lightIOReal;

    public LightIOReal(RealAllocator lightIOReal, int length) {
        this.length = length;

        this.lightIOReal = lightIOReal;
        startDex = lightIOReal.allocateLength(length);
    }

    @Override
    public void setHSV(int index, int hue, int saturation, int lightness) {
        lightIOReal.setHSV(startDex + index, hue, saturation, lightness);
    }

    @Override
    public int getLength() {
        return length;
    }

    @Override
    public Color8Bit getLED8Bit(int index) {
        return lightIOReal.getColor8Bit(startDex + index);
    }

    @Override
    public void periodic() {
        lightIOReal.periodic();
    }

}
