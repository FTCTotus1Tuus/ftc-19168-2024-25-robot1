package org.firstinspires.ftc.teamcode.team.DarienFSM.Subsystems;

public class IntakeFSM {
    // INTAKE FINITE STATE MACHINE
    public enum IntakeMechanismState {
        READY_FOR_SAMPLE_MECHANISM,
        EXTENDING_TO_SAMPLE,
        READY_TO_GRAB_SAMPLE,
        GRABBING_SAMPLE,
        RETRACTING_INTAKE_TO_DROP,
        READY_FOR_SAMPLE_DROP,
        SAMPLE_DROP
    }

    IntakeMechanismState intakeMechanismState = IntakeMechanismState.READY_FOR_SAMPLE_MECHANISM;

    public void loop() {
        switch (intakeMechanismState) {
            case READY_FOR_SAMPLE_MECHANISM:
                if (true) {

                }
                break;
            case EXTENDING_TO_SAMPLE:
                if (true) {
                }
                break;
            case READY_TO_GRAB_SAMPLE:
                if (true) {
                }
                break;
            case GRABBING_SAMPLE:
                break;
            case RETRACTING_INTAKE_TO_DROP:
                if (true) {
                }
                break;
            case READY_FOR_SAMPLE_DROP:
                if (true) {
                }
                break;
            case SAMPLE_DROP:
                if (true) {
                }
                break;
            default:
                // Throw an exception error since we should never go into the default case.
        }
    }
}
