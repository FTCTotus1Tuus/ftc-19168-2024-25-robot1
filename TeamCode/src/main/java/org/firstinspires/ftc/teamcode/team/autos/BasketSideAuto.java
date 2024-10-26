package org.firstinspires.ftc.teamcode.team.autos;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.team.DarienOpModeAuto;

@Autonomous
public class BasketSideAuto extends DarienOpModeAuto {

    public void runOpMode() {
        initControls();

        waitForStart();


        // put into one function with all basic servo settings
        setSpecimenClaw("closed");

        //From start intake towards wall
        moveXY(0, -33, normalPower);
        setVerticalSlide("2nd bar below", verticalSlidePower);
        waitForMotors();
        setVerticalSlide("2nd bar place", verticalSlidePower);
        waitForArm();
        setSpecimenClaw("open");

        //go to first sample and attempt to pick it up
        moveXY(0, 5, normalPower);
        waitForMotors();
        moveXY(26, 0, normalPower);
        waitForMotors();
        autoRotate(3*PI / 2, normalPower);
        moveXY(10, 0, normalPower); // at position to pick up sample
        waitForMotors();
        setIntakeWrist("down");
        startIntake();
        moveXY(0, 10, 0.15);
        while (!isIntakeSensorOn()) {}
        stopIntake();
        placeSampleInBucket();
        moveXY(-24, 0, normalPower);
        sleep(250);
        setVerticalSlide("basket high", verticalSlidePower);
        waitForMotors();
        autoRotate(3*PI / 4, normalPower);
        moveXY(0,12, normalPower);
        waitForMotors();
        setBucketPosition("drop");
        sleep(250);
        setBucketPosition("carry");
        moveXY(0, -12, normalPower);
        setVerticalSlide("low", verticalSlidePower);


    }

}
