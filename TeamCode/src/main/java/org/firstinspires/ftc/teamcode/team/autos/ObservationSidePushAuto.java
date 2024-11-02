package org.firstinspires.ftc.teamcode.team.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.team.DarienOpModeAuto;

@Autonomous
public class ObservationSidePushAuto extends DarienOpModeAuto {
    public void runOpMode() {
        initControls();

        waitForStart();
        // move to chambers with preloaded specimen
        setSpecimenClaw("closed");
        setSpecimenWrist("place");
        moveXY(0, 30, normalPower);
        setVerticalSlide("high chamber below", verticalSlidePower);
        waitForMotors();
        waitForArm();
        // place specimen on high chamber
        setVerticalSlide("high chamber place", verticalSlidePower);
        waitForArm();
        setSpecimenWrist("pickup");
        // push samples to observation zone
        // park
        /*moveXY(0, 28, normalPower);
        setVerticalSlide("2nd bar below", verticalSlidePower);
        waitForMotors();

        setVerticalSlide("2nd bar place", verticalSlidePower);*/
    }
}
