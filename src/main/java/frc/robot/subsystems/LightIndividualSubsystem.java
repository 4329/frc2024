package frc.robot.subsystems;

import java.awt.Color;
import java.util.List;
import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightIndividualSubsystem extends SubsystemBase {

    private AddressableLED addressableLED;
    private AddressableLEDBuffer addressableLEDBuffer;
    private int hue;
    private int saturation = 255;
    private int brightness = 128;

    private List<Color8Bit> savedColors;

    public LightIndividualSubsystem() {
        addressableLED = new AddressableLED(1);
        addressableLEDBuffer = new AddressableLEDBuffer(120);
        addressableLED.setLength(addressableLEDBuffer.getLength());
        savedColors = new ArrayList<>(addressableLEDBuffer.getLength());
        addressableLED.setData(addressableLEDBuffer);
        addressableLED.start();

        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (alliance.get().equals(DriverStation.Alliance.Red)) {
                hue = 0;
                System.out.println("000000000000000000000000000000000000000000000000000000000000000000000000");
            } else {
                hue = 120;
                System.out.println("120........................................................");
            }
        } else {
            hue = 60;
            System.out.println("60%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
        }
    }

    public void beforeMatchColors() {
        for (int i = 0; i < addressableLEDBuffer.getLength(); i++) {
            addressableLEDBuffer.setHSV(i, hue, 255, 150);
        }
    }

    public void blank() {
        brightness = 0;
        for (int i = 0; i < addressableLEDBuffer.getLength(); i++) {
            Color8Bit color = addressableLEDBuffer.getLED8Bit(i);
            savedColors.add(i, color);
            addressableLEDBuffer.setHSV(i, 0, 0, brightness);
        }
    }

    public void lightsOn() {
        brightness = 128;
        for (int i = 0; i < addressableLEDBuffer.getLength(); i++) {
            Color8Bit color = savedColors.get(i);
            if (color != null) {
                float[] hsvVals = Color.RGBtoHSB(color.red, color.green, color.blue, null);
                addressableLEDBuffer.setHSV(i, (int) (hsvVals[0] * 180.0), (int) (hsvVals[1] * 255.0), brightness);
            } else {
                addressableLEDBuffer.setHSV(i, 0, 0, brightness);
            }
        }
    }

    public void progressBarColorrrr(double progress, int hue, int saturation) {
        int length = (int) (addressableLEDBuffer.getLength() * progress);
        for (int i = 0; i <= length; i++) {
            addressableLEDBuffer.setHSV(i, 60, saturation, 150);
        }
    }

    public void loadingBarColor(int start, int stop, int hue, int saturation) {
        for (int i = 0; i < addressableLEDBuffer.getLength(); i++) {
            if (i > start && i < stop) {
                addressableLEDBuffer.setHSV(i, hue, saturation, 150);
            } else {
                addressableLEDBuffer.setHSV(i, 0, 0, 0);
            }
        }
    }

    public void setColor(int hue, int saturation) {
        for (int i = 0; i < addressableLEDBuffer.getLength(); i++) {
            addressableLEDBuffer.setHSV(i, hue, saturation, brightness);
        }
    }

    public void setBlack() {
        for (int i = 0; i < addressableLEDBuffer.getLength(); i++) {
            addressableLEDBuffer.setHSV(i, 0, 0, 0);
        }
    }

    public void rainbow() {
        // For every pixel
        for (var i = 0; i < addressableLEDBuffer.getLength(); i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            final int hues = (int) (hue + (i * 180 / addressableLEDBuffer.getLength())) % 180;
            // Set the value
            addressableLEDBuffer.setHSV(i, hues, 255, 128);
        }
        // Increase by to make the rainbow "move"
        hue += 1;
        // Check bounds
        hue %= 180;

    }

    @Override
    public void periodic() {
        addressableLED.setData(addressableLEDBuffer);
        // System.out.println("periodic is runing____________________________________");
    }

}
