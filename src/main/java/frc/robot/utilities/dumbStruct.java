package frc.robot.utilities;

import edu.wpi.first.util.struct.Struct;
import java.nio.ByteBuffer;

public class dumbStruct implements Struct<dumb> {

  @Override
  public Class<dumb> getTypeClass() {
    return dumb.class;
  }

  @Override
  public String getTypeString() {
    return "struct:dumb";
  }

  @Override
  public int getSize() {
    return 4;
  }

  @Override
  public String getSchema() {
    return "int a;";
  }

  @Override
  public dumb unpack(ByteBuffer bb) {
    int a = bb.getInt();
    return new dumb(a);
  }

  @Override
  public void pack(ByteBuffer bb, dumb value) {
    bb.putInt(value.a);
  }
}
