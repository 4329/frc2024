package frc.robot.utilities;

public enum ArmAngle {
  ZERO(0),
  SUB(0),
  HORIZONTAL(3.65),
  INTAKE(1.2),
  ARMAMP(5.7),
  FULL(6);

  private double value;

  private ArmAngle(double bob) {

    this.value = bob;
  }

  public double getValue() {

    return value;
  }
}
