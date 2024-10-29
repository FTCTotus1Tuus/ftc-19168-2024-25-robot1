package org.firstinspires.ftc.teamcode.team.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.team.DarienOpModeAuto;

@Autonomous
public class BasketSidePushAuto extends DarienOpModeAuto {
    public void runOpMode() {
        initControls();

        waitForStart();

        setVerticalSlide("basket high", verticalSlidePower);
        sleep(5500);
        moveXY(0, -16, normalPower);
        waitForArm();
        waitForMotors();

        setBucketPosition("drop");
        sleep(1000);
        setBucketPosition("carry");
// sample number 1
        moveXY(-36, 22.5, normalPower);
        setVerticalSlide("low",verticalSlidePower);
        waitForMotors();
        moveXY(-15, 0, normalPower);
        waitForMotors();
        moveXY(0,-12, normalPower);
        waitForMotors();
        autoRotate(PI/2, normalPower);
        moveXY(-6, -48, normalPower);
        waitForMotors();
        // sample number 2
        moveXY(10, 48, normalPower);
        waitForMotors();
        autoRotate(PI/2, normalPower);
        moveXY(-13, 0, normalPower);
        waitForMotors();
        moveXY(3, -48, normalPower);
        waitForMotors();
        // sample number 3
        moveXY(5, 48, normalPower);
        waitForMotors();
        autoRotate(PI/2, normalPower);
        moveXY(-17, 0, normalPower); //untested change
        waitForMotors();
        moveXY(0, -50, normalPower);
        waitForMotors();
        // start level 1 ascent
        moveXY(0, 60, normalPower);
        waitForMotors();
        autoRotate(PI, normalPower);
        setVerticalSlide("1st bar place", verticalSlidePower);
        moveXY(0, -48, normalPower); //untested change
        waitForMotors();
        waitForArm();



    }
}
