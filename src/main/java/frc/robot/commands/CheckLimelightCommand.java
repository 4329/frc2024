package frc.robot.commands;

import java.io.IOException;
import java.net.HttpURLConnection;
import java.net.URL;

import edu.wpi.first.wpilibj2.command.Command;

public class CheckLimelightCommand extends Command {

    private URL limelihURL;
    private boolean connected;
    private boolean setConnected;
    private boolean end;

    public boolean getConnected() {
        return connected;
    }

    public CheckLimelightCommand() {
        try {
            limelihURL = new URL("http://10.43.29.11:5807/results");
        } catch (IOException ioException) {
            ioException.printStackTrace();
        }
    }


    @Override
    public void initialize() {
        System.out.println("sdfafads_____________________________________________________________________________________________________________________________________________________fdsa");
        end = false;
        new Thread(() -> {
            try {
                setConnected = true;
                HttpURLConnection con = (HttpURLConnection) limelihURL.openConnection();
                con.setConnectTimeout(2000);
                con.setRequestMethod("GET");
                con.getResponseCode();
            } catch (IOException e) {
                e.printStackTrace();
                setConnected = false;
            }
            connected = setConnected;
            end = true;
        }).start();
    }

    @Override
    public boolean isFinished() {
        return end;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
        System.out.println(connected);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
