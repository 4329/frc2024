package frc.robot.utilities;

public enum ElevatorSetpoints {

    ZERO(0), ONEHUNDRED(100);

    private float value;

    private ElevatorSetpoints(float jim) {

        this.value = jim;

    }

    public float getValue() {

        return value;

    }
}