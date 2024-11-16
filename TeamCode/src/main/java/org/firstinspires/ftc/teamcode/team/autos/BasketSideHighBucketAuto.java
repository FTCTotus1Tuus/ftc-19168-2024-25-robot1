package org.firstinspires.ftc.teamcode.team.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.team.DarienOpModeAuto;

@Autonomous
public class BasketSideHighBucketAuto extends DarienOpModeAuto {
    public void runOpMode() {
        initControls();

        waitForStart();
        intakeSlide.setPower(-0.1);
        setIntakeWrist("slightly up");
        setBucketPosition("carry");
        setVerticalSlide("basket high", 0.8);
        sleep(1800);
        moveXY(-3, -16, normalPower);
        waitForArm();
        waitForMotors();

        setBucketPosition("drop");
        sleep(1000);
        setBucketPosition("carry");
        sleep(200);
// sample number 1
        moveXY(-10, 1, normalPower);
        sleep(500);
        setVerticalSlide("low", 0.8);
        waitForMotors();
        encoderRotate(PI / 2, normalPower, false);
        waitForMotors();
        setIntakeWrist("down");
        startIntake();
        moveXY(0, 11, 0.1); // pickup sample 1
        waitForMotors();
        setIntakeWrist("up");
        sleep(150);
        moveXY(5, -11, normalPower);
        sleep(200);
        reverseIntake();
        sleep(500);
        setVerticalSlide("basket high", 1); // raise arm for sample 1
        waitForMotors();
        stopIntake();
        encoderRotate(PI / 4, normalPower, true);
        waitForMotors();
        moveXY(0, -14.5, normalPower);
        waitForArm();
        waitForMotors();
        setBucketPosition("drop"); // place sample 1
        sleep(1000);
        moveXY(0, 10, normalPower);
        setBucketPosition("carry");
        setVerticalSlide("low", 0.8);
        //start sample 2
        waitForMotors();
        encoderRotate(PI / 4, normalPower, false);
        waitForMotors();
        moveXY(-10, 0, normalPower);
        waitForMotors();
        setIntakeWrist("down");
        startIntake();
        moveXY(0, 10, 0.1); // pickup sample 2
        waitForMotors();
        setIntakeWrist("up");
        sleep(150);
        moveXY(0, -10, 0.1);
        reverseIntake();
        sleep(300);
        setVerticalSlide("basket high", 1); // raise arm sample 2
        stopIntake();
        waitForMotors();
        moveXY(15, 0, normalPower);
        encoderRotate(PI / 4, normalPower, true);
        waitForMotors();
        moveXY(0, -16, normalPower);
        waitForMotors();
        setBucketPosition("drop"); // place sample 2
        sleep(800);
        moveXY(0, 10, normalPower);
        setBucketPosition("carry");
        sleep(400);
        setVerticalSlide("low", 0.8); // start park
        encoderRotate((PI - PI / 4), normalPower, true);
        waitForMotors();
        moveXY(0, -40, normalPower);
        sleep(2000);
        setVerticalSlide("1st bar place", 0.8);
        waitForMotors();
        encoderRotate((PI / 2) - 0.15, normalPower, true);
        waitForMotors();
        moveXY(0, -20, normalPower); // park
        waitForMotors();

    }
}
