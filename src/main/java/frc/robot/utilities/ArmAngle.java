package frc.robot.utilities;

public enum ArmAngle {

    ZERO(0), HORIZONTAL(3.5), INTAKE(0), ARMAMP(5), FULL(5.2);

    private double value;

    private ArmAngle(double bob) {

        this.value = bob;

    }

    public double getValue() {

        return value;

    }

}
