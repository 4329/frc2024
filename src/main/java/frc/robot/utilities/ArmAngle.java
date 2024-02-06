package frc.robot.utilities;

public enum ArmAngle {

    ZERO(0), HORIZONTAL(13), INTAKE(7), FULL(23);

    private float value;

    private ArmAngle(float bob) {

        this.value = bob;

    }

    public float getValue() {

        return value;

    }

}
