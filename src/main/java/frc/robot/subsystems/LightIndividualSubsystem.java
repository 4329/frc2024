package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightIndividualSubsystem extends SubsystemBase {

    private AddressableLED addressableLED;
    private AddressableLEDBuffer addressableLEDBuffer;
    private int hue;
    private int saturation;
    private int lightness;

    public LightIndividualSubsystem() {
        lightness = 0;
        saturation = 255;
        addressableLED = new AddressableLED(1);
        addressableLEDBuffer = new AddressableLEDBuffer(60);
        addressableLED.setLength(addressableLEDBuffer.getLength());
        addressableLED.setData(addressableLEDBuffer);
        addressableLED.start();

        
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if(alliance.isPresent()){
            if(alliance.get().equals(DriverStation.Alliance.Red)){
                hue = 0;
            }
            else{
                hue = 120;
            }
        } 
        else{hue = 60;};
    }

    public void beforeMatchColors() {
        for (int i = 0; i < addressableLEDBuffer.getLength(); i++) {
            addressableLEDBuffer.setHSV(i, hue, 255, 150);
        }

        addressableLED.setData(addressableLEDBuffer);
    }

    public void blank() {
        for (int i = 0; i < addressableLEDBuffer.getLength(); i++) {
            addressableLEDBuffer.setHSV(i, 0, 0, 0);
        }

        addressableLED.setData(addressableLEDBuffer);
    }

    public void progressBar(double progress, int hue, int saturation, int brightness) {
        int length = (int) (addressableLEDBuffer.getLength() * progress);
        for (int i = 0; i < length; i++) {
            addressableLEDBuffer.setHSV(i, hue, saturation, brightness);
        }

        addressableLED.setData(addressableLEDBuffer);
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
        addressableLED.setData(addressableLEDBuffer);

    }

    @Override
    public void periodic() {
        // bufferColors(hue, saturation, lightness);
    }

}
