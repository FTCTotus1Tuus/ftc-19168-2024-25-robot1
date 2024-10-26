package org.firstinspires.ftc.teamcode.team.autos;

import org.firstinspires.ftc.teamcode.team.DarienOpModeAuto;

public class ObservationSidePushAuto extends DarienOpModeAuto {
    public void runOpMode() {
        moveXY(0, 28, normalPower);
        setVerticalSlide("2nd bar below", verticalSlidePower);
        waitForMotors();

        setVerticalSlide("2nd bar place", verticalSlidePower);
    }
}
