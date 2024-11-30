package org.firstinspires.ftc.teamcode.team.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.team.DarienOpModeAuto;

@Autonomous
public class ObservationSidePushAuto extends DarienOpModeAuto {
    public void runOpMode() {
        initControls();

        waitForStart();
        // specimen 1
        // move to chambers with preloaded specimen
        setSpecimenClaw("closed");
        setSpecimenWrist("place");
        moveToPosition(-5, 33, normalPower);
        setVerticalSlide("high chamber below", verticalSlidePower);
        waitForMotors();
        waitForArm();

        // place specimen 1 on high chamber
        setVerticalSlide("high chamber place", verticalSlidePower);
        waitForArm();
        // release the specimen and move the specimenWrist away to avoid hitting the submersible when strafing.
        setSpecimenClaw("open");
        setSpecimenWrist("pickup");

        // specimen 2
        // move to observation zone and get specimen 2
        moveToPosition(30, 0, normalPower);
        setVerticalSlide("low", verticalSlidePower);
        waitForMotors();
        waitForArm();
        setSpecimenClaw("closed");
        setSpecimenWrist("place");
        // move to chambers
        moveToPosition(-5, 33, normalPower);
        setVerticalSlide("high chamber below", verticalSlidePower);
        waitForMotors();
        waitForArm();
        // place specimen 2 on high chamber
        setVerticalSlide("high chamber place", verticalSlidePower);
        waitForArm();
        // release the specimen and move the specimenWrist away to avoid hitting the submersible when strafing.
        setSpecimenClaw("open");
        setSpecimenWrist("pickup");
        setVerticalSlide("low", verticalSlidePower);
        moveToPosition(-5, 28, normalPower);
        waitForMotors();


        // push samples to observation zone
        // sample 1
        moveToPosition(28, 28, normalPower);
        waitForMotors();
        moveToPosition(28, 53, normalPower);
        waitForMotors();
        moveToPosition(37, 53, normalPower);
        waitForMotors();
        moveToPosition(37, 8, normalPower);
        waitForMotors();
        //sample 2
        moveToPosition(37, 53, normalPower);
        waitForMotors();
        moveToPosition(46, 53, normalPower);
        waitForMotors();
        moveToPosition(46, 8, normalPower);
        waitForMotors();
        //sample 3
        moveToPosition(46, 53, normalPower);
        waitForMotors();
        moveToPosition(55, 53, normalPower);
        waitForMotors();
        moveToPosition(55, 5, normalPower);
        waitForMotors();
        //park
        moveToPosition(55, 6, normalPower);
        waitForMotors();

        // park
        /* moveToPosition(0, 28, normalPower);
        setVerticalSlide("2nd bar below", verticalSlidePower);
        waitForMotors();

        setVerticalSlide("2nd bar place", verticalSlidePower);*/
    }
}
