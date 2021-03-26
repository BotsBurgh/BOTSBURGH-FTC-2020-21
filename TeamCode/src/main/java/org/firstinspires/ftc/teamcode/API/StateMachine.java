package org.firstinspires.ftc.teamcode.API;

public class StateMachine {
    public enum State {
        ARMIN,
        ARMOUT,
        DRIVE
    }
    private static State currentState;

    public State getCurrentState() {
        return currentState;
    }

    public void setCurrentState(State state) {
        currentState = state;
    }
}
