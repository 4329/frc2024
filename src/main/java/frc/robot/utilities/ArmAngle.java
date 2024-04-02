package frc.robot.utilities;

public enum ArmAngle {
  ZERO(0),
  SUB(0),
  HORIZONTAL(14.2),
  INTAKE(2.75),
  INDEXSOURCE(14.2),
  SHOOTERSOURCE(4),
  AMPDEX(14.2),
  SHOOTERARMAMP(22),
  FULL(22);

  private double value;

  private ArmAngle(double bob) {

    this.value = bob;
  }

  public double getValue() {

    return value;
  }
}
