package frc.robot.utilities;

public enum ArmAngle {
  ZERO(0),
  SUB(0),
  HORIZONTAL(14.2),
  INTAKE(2.75),
  ARMAMP(22),
  FULL(22);

  private double value;

  private ArmAngle(double bob) {

    this.value = bob;
  }

  public double getValue() {

    return value;
  }
}
