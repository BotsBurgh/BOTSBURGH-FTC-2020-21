package org.firstinspires.ftc.teamcode.API;

/**
 * State machine to ease synchronize and prevents bugs with robot movements
 */
public class StateMachine {
    public enum State {
        ARMIN,
        ARMOUT,
        DRIVE
    }

    private static State currentState;

    /**
     * Returns what the robot is doing at the current instant
     * @return the state of the robot's current action
     */
    public State getCurrentState() {
        return currentState;
    }

    /**
     * Modifies the state of the robot based on the user's command
     * @param state the new state of robot based on the user's command
     */
    public void setCurrentState(State state) {
        currentState = state;
    }
}
