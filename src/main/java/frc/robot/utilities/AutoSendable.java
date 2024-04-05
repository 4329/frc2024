package frc.robot.utilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.proto.Geometry2D.ProtobufPose2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import us.hebi.quickbuf.InvalidProtocolBufferException;

public class AutoSendable implements Sendable {
  public Pose2d pose;
  ProtobufPose2d a;

  public AutoSendable(Pose2d pose) {
    this.pose = pose;
    a = ProtobufPose2d.newInstance();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("PlannerDisplayType");
    builder.addRawProperty("pose", "Pose2D", this::getPose, this::setPose);
  }

  public byte[] getPose() {
    Pose2d.proto.pack(a, pose);
    return a.toByteArray();
  }

  public void setPose(byte[] newPose) {
    try {
      ProtobufPose2d tmp = ProtobufPose2d.parseFrom(newPose);
      pose = Pose2d.proto.unpack(tmp);
    } catch (InvalidProtocolBufferException e) {
      pose = null;
      throw new RuntimeException(e);
    }
  }
}
