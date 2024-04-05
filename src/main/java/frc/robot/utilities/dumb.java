package frc.robot.utilities;

import edu.wpi.first.util.struct.StructSerializable;

public class dumb implements StructSerializable {
  int a;

  public dumb(int a) {
    this.a = a;
  }

  public static final dumbStruct structDumb = new dumbStruct();
}
