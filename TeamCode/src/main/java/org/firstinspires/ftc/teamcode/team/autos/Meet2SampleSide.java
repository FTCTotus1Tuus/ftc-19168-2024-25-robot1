package org.firstinspires.ftc.teamcode.team.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.team.DarienOpModeAuto;

@Autonomous
public class Meet2SampleSide extends DarienOpModeAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        initControls();

        waitForStart();
        intakeSlide.setPower(-0.1);
        setIntakeWrist("slightly up");
        setBucketPosition("carry");
        setSpecimenWrist("place");
        setVerticalSlide("basket high", 0.8);
        sleep(1800);
        moveToPosition(-3, -17.5, normalPower);
        waitForArm();
        waitForMotors();

        setBucketPosition("drop");
        sleep(1000);
        setBucketPosition("carry");
        sleep(200);
// sample number 1
        moveToPosition(-15, -15, normalPower);
        sleep(500);
        setVerticalSlide("low", 0.8);
        waitForMotors();
        autoRotate(90, normalPower);
        waitForMotors();
        setIntakeWrist("down");
        startIntake();
        moveToPosition(-28, -15, 0.1); // pickup sample 1
        waitForMotors();
        setIntakeWrist("up");
        sleep(300);
        autoRotate(0, normalPower);
        moveToPosition(0, -9.5, normalPower);
        sleep(300);
        reverseIntake();
        sleep(500);
        setVerticalSlide("basket high", 1); // raise arm for sample 1
        waitForMotors();
        stopIntake();
        moveToPosition(0, -18, normalPower);
        waitForArm();
        waitForMotors();
        setBucketPosition("drop"); // place sample 1
        sleep(1000);
        //start sample 2
        moveToPosition(-15, -25, normalPower);
        setBucketPosition("carry");
        setVerticalSlide("low", 0.8);
        waitForMotors();
        autoRotate(90, normalPower);
        waitForMotors();
        setIntakeWrist("down");
        startIntake();
        moveToPosition(-28, -25, 0.1);
        waitForMotors();
        sleep(150);
        setIntakeWrist("up");
        sleep(400);
        autoRotate(0, normalPower);
        moveToPosition(0, -9.5, normalPower);
        reverseIntake();
        sleep(500);
        setVerticalSlide("basket high", 1); // raise arm sample 2
        stopIntake();
        waitForMotors();
        moveToPosition(0, -18, normalPower);
        waitForMotors();
        setBucketPosition("drop"); // place sample 2
        sleep(800);
        moveToPosition(-15.5, -9.5, normalPower);
        waitForMotors();
    }
}
