// /**
//  * ChatClientEndpoint.java
//  * http://programmingforliving.com
//  */
package frc.robot.utilities;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;
import java.net.URI;
import java.net.http.HttpClient;

public class SocketEndpoint {

  private WebsocketListener listener;
  private ObjectMapper objectMapper;

  public SocketEndpoint() {
    HttpClient httpClient = HttpClient.newHttpClient();
    listener = new WebsocketListener();
    httpClient.newWebSocketBuilder().buildAsync(URI.create("ws://10.43.29.11:5806"), listener);

    objectMapper = new ObjectMapper();
    objectMapper.configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false);
  }

  public LimelightHelpers.LimelightResults getResults() {
    try {
      String asdfs = listener.getOutput();
      return objectMapper.readValue(asdfs, LimelightHelpers.LimelightResults.class);
    } catch (JsonProcessingException e) {
      System.err.println("lljson error: " + e.getMessage());
      return new LimelightHelpers.LimelightResults();
    }
  }
}
