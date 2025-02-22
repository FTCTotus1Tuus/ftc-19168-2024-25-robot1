package org.firstinspires.ftc.teamcode.team.DarienFSM.Subsystems;

public class ScoringFSM {

    // FINITE STATE MACHINE
    private enum ScoringSubsystemState {
        START,
        READY_FOR_SPECIMEN_PICKUP,
        PICKING_UP_SPECIMEN,
        EXTENDING_TO_READY_TO_CLIP_HIGH_CHAMBER,
        READY_TO_CLIP_HIGH_CHAMBER,
        EXTENDING_TO_CLIP_HIGH_CHAMBER,
        CLIPPED_HIGH_CHAMBER,
        READY_FOR_SAMPLE_PICKUP,
        READY_FOR_SAMPLE_DROP,
        SAMPLE_DROP
    }

    // We declare a variable to persist the state between loop() calls
    ScoringSubsystemState scoringSubsystemState = ScoringSubsystemState.START;

    public void init() {
        // Reset any component timers first.

        // hardware initialization code goes here
        // this needs to correspond with the configuration used
    }

    public void loop() {
        switch (scoringSubsystemState) {
            case READY_FOR_SPECIMEN_PICKUP:
                if (true) {

                }
                break;
            case PICKING_UP_SPECIMEN:
                if (true) {

                }
                break;
            case EXTENDING_TO_READY_TO_CLIP_HIGH_CHAMBER:
                if (true) {
//                   setVerticalSlide("high chamber below", verticalSlidePower);
                }
                break;
            case READY_TO_CLIP_HIGH_CHAMBER:
//               if (setVerticalSlide("high chamber below", verticalSlidePower)) {
//                    scoringSubsystemState = ScoringSubsystemState.EXTENDING_TO_CLIP_HIGH_CHAMBER;
//               }
                break;
            case EXTENDING_TO_CLIP_HIGH_CHAMBER:
                if (true) {

                }
                break;
            case CLIPPED_HIGH_CHAMBER:
                if (true) {

                }
                break;
            case READY_FOR_SAMPLE_PICKUP:
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
