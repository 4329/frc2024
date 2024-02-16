package frc.robot.commands.drive;

import java.net.URL;
import java.nio.Buffer;
import java.nio.charset.MalformedInputException;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.HttpURLConnection;
import java.net.SocketTimeoutException;

import edu.wpi.first.wpilibj2.command.Command;

public class LimlihCommand extends Command {

    private URL url;

    public LimlihCommand() {
        System.out.println("adfsadfsdf");
    }
    
    public boolean connected;
    
    @Override
    public void initialize() {
        try {
            url = new URL("http://10.43.29.11:5807/results");
        } catch (IOException ioException) {
            ioException.printStackTrace();
            // throw new RuntimeException(ioException);
        }
        try {
            connected = true;
            HttpURLConnection con = (HttpURLConnection) url.openConnection();
            con.setConnectTimeout(3000);
            con.setRequestMethod("GET");
            con.getResponseCode();
        } catch (SocketTimeoutException socketTimeoutException) {
            socketTimeoutException.printStackTrace();
            connected = false;
            // throw new RuntimeException(socketTimeoutException);
        } catch (IOException ioException) {
            ioException.printStackTrace();
            connected = false;
        
            // throw new RuntimeException(ioException);
        }
        
    }

    @Override
    public void execute() {
        System.out.println(connected);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
