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
        moveXY(-5, 33, normalPower);
        setVerticalSlide("high chamber below", verticalSlidePower);
        waitForMotors();
        waitForArm();

        // place specimen 1 on high chamber
        setVerticalSlide("high chamber place", verticalSlidePower);
        waitForArm();
        // release the specimen and move the specimenWrist away to avoid hitting the submersible when strafing.
        setSpecimenWrist("pickup");
        setSpecimenClaw("open");
        setVerticalSlide("low", verticalSlidePower);
        moveXY(0, -5, normalPower);
        waitForMotors();
        setSpecimenWrist("place");
        // specimen 2
        moveXY(normalPower);
        setSpecimenWrist("pickup");
        setSpecimenClaw("open");
        waitForMotors();
        setSpecimenClaw("closed");
        moveXY(normalPower);
        setSpecimenWrist("place");
        setVerticalSlide("high chamber place", verticalSlidePower);
        moveXY(normalPower);
        waitForMotors();
        waitForArm();


        // push samples to observation zone
        // sample 1
        moveXY(33, 0, normalPower);
        waitForMotors();
        moveXY(0, 25, normalPower);
        waitForMotors();
        moveXY(9, 0, normalPower);
        waitForMotors();
        moveXY(0, -45, normalPower);
        waitForMotors();
        //sample 2
        moveXY(0, 45, normalPower);
        waitForMotors();
        moveXY(9, 0, normalPower);
        waitForMotors();
        moveXY(0, -45, normalPower);
        waitForMotors();
        //sample 3
        moveXY(0, 45, normalPower);
        waitForMotors();
        moveXY(9, 0, normalPower);
        waitForMotors();
        moveXY(0, -48, normalPower);
        waitForMotors();
        //park
        moveXY(0, 1, normalPower);
        waitForMotors();

        // park
        /*moveXY(0, 28, normalPower);
        setVerticalSlide("2nd bar below", verticalSlidePower);
        waitForMotors();

        setVerticalSlide("2nd bar place", verticalSlidePower);*/
    }
}
