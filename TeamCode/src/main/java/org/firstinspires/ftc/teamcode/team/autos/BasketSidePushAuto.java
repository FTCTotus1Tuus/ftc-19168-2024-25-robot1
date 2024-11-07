package org.firstinspires.ftc.teamcode.team.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.team.DarienOpModeAuto;

@Autonomous
public class BasketSidePushAuto extends DarienOpModeAuto {
    public void runOpMode() {
        initControls();

        waitForStart();
        intakeSlide.setPower(-0.1);
        setIntakeWrist("slightly up");
        setBucketPosition("carry");
        setVerticalSlide("basket high", 0.8);
        sleep(1800);
        moveXY(0, -16, normalPower);
        waitForArm();
        waitForMotors();

        setBucketPosition("drop");
        sleep(1000);
        setBucketPosition("carry");
        sleep(200);
// sample number 1
        moveXY(-10, 21, normalPower);
        sleep(200);
        setVerticalSlide("low", 0.3);
        waitForMotors();
        encoderRotate(PI / 2, normalPower, false);
        waitForMotors();
        moveXY(0, 37, normalPower);
        waitForMotors();
        moveXY(-11, 0, normalPower);
        waitForMotors();
        moveXY(-3, -46, normalPower);
        waitForMotors();
        // sample number 2
        moveXY(3, 48, normalPower);
        waitForMotors();
//        autoRotate(PI/2, normalPower);
        moveXY(-13, 0, normalPower);
        waitForMotors();
        moveXY(3, -47, 0.5);
        waitForMotors();
        // sample number 3
//        moveXY(1, 47, normalPower);
//        waitForMotors();
////        autoRotate(PI/2, normalPower);
//        moveXY(-12, 0, normalPower);
//        waitForMotors();
//        moveXY(-1, -40, normalPower);
//        waitForMotors();
        // start level 1 ascent
        moveXY(0, 48, normalPower); // untested
        waitForMotors();
        encoderRotate(PI / 2, normalPower, false);
        waitForMotors();
        setVerticalSlide("1st bar place", 0.8);
        moveXY(0, -44, normalPower);
        waitForMotors();
        waitForArm();


    }
}
