package org.firstinspires.ftc.teamcode.team.autos;

import org.firstinspires.ftc.teamcode.team.DarienOpModeAuto;

public class BasketSidePushAuto extends DarienOpModeAuto {
    public void runOpMode() {
        initControls();

        waitForStart();

        setVerticalSlide("basket high", verticalSlidePower);
        sleep(1000);
        moveXY(0, 5, normalPower);
        waitForArm();
        waitForMotors();

        setBucketPosition("drop");
        sleep(500);
        setBucketPosition("carry");

        moveXY(36, -12, normalPower);
        waitForMotors();
        moveXY(0, 6, normalPower);
        waitForMotors();
        autoRotate(PI/2, normalPower);
        moveXY(6, 36, normalPower);
        waitForMotors();
        moveXY(-6, -48, normalPower);
        waitForMotors();
        moveXY(0, 60, normalPower);
        waitForMotors();
        moveXY(0, -60, normalPower);
        waitForMotors();
        moveXY(12, 0, normalPower);
        waitForMotors();
        moveXY(0, 60, normalPower);
        waitForMotors();
        moveXY(0, -60, normalPower);
        waitForMotors();
        autoRotate(PI, normalPower);
        setVerticalSlide("1st bar place", verticalSlidePower);
        moveXY(0, 24, normalPower);
        waitForMotors();
        waitForArm();



    }
}
