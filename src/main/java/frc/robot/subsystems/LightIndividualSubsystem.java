package frc.robot.subsystems;

import java.awt.*;
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

    public LightIndividualSubsystem() {
        addressableLED = new AddressableLED(1);
        addressableLEDBuffer = new AddressableLEDBuffer(60);
        addressableLED.setLength(addressableLEDBuffer.getLength());
        addressableLED.setData(addressableLEDBuffer);
        addressableLED.start();

        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (alliance.get().equals(DriverStation.Alliance.Red)) {
                hue = 0;
            } else {
                hue = 120;
            }
        } else {
            hue = 60;
        }
    }

    public void beforeMatchColors() {
        for (int i = 0; i < addressableLEDBuffer.getLength(); i++) {
            addressableLEDBuffer.setHSV(i, hue, 255, 150);
        }
    }

    private void changeBrightness(int newBrightness) {
        brightness = newBrightness;
        for (int i = 0; i < addressableLEDBuffer.getLength(); i++) {
            Color8Bit color = addressableLEDBuffer.getLED8Bit(i);
            float[] hsvVals = Color.RGBtoHSB(color.red, color.green, color.blue, null);
            addressableLEDBuffer.setHSV(i, (int)(hsvVals[0] * 180.0), (int)(hsvVals[1] * 255.0), brightness);
        }
    }

    public void blank() {
        changeBrightness(0);
    }

    public void lightsOn(){
        changeBrightness(128);
    }

    public void progressBarColorrrr(double progress, int hue, int saturation) {
        int length = (int) (addressableLEDBuffer.getLength() * progress);
        for (int i = 0; i < length; i++) {
            addressableLEDBuffer.setHSV(i, hue, saturation, brightness);
        }
    }

    public void setColor(int hue, int saturation) {
        for (int i = 0; i < addressableLEDBuffer.getLength(); i++) {
            addressableLEDBuffer.setHSV(i, hue, saturation, brightness);
        }
    }

    public void rainbow() {
        // For every pixel
        for (var i = 0; i < addressableLEDBuffer.getLength(); i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            final int hues = (int) (hue + (i * 180 / addressableLEDBuffer.getLength())) % 180;
            // Set the value
            addressableLEDBuffer.setHSV(i, hues, 255, brightness);
        }
        // Increase by to make the rainbow "move"
        hue += 1;
        // Check bounds
        hue %= 180;

    }

    @Override
    public void periodic() {
        addressableLED.setData(addressableLEDBuffer);
    }

}
