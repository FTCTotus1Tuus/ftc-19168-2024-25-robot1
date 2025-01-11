package org.firstinspires.ftc.teamcode.team.autos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.team.DarienOpModeAuto;

@Disabled
@Config
@Autonomous
public class Meet3SampleSideSebastianIsCrazy extends DarienOpModeAuto {
    public double normalPower = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        initControls();

        waitForStart();
        // intakeSlide.setPower(-0.1);
        setIntakeWrist("slightly up");
        setBucketPosition("carry");
        setSpecimenWrist("place");
        setVerticalSlide("basket high", 1);
        sleep(800); // TODO
        moveToPosition(-3, -17.5, 0.3);// approach slowly
        waitForArm();
        waitForMotors(1);
        setBucketPosition("drop");
        sleep(600); //TODO

        // sample number 1
        moveToPosition(-15, -15, 0.4);
        sleep(200);
        setBucketPosition("carry");
        setVerticalSlide("low", 0.8);
        waitForMotors(2);
        autoRotate(90, 0.4);
        waitForMotors(0.8);
//        intakeSlide.setPower(0.1);
//        sleep(200);
//        intakeSlide.setPower(0);
        setIntakeWrist("down");
        startIntake();
//        intakeSlide.setPower(-0.1);
//        sleep(200);
//        intakeSlide.setPower(0);
        moveToPosition(-28.5, -15, 0.1); // pickup sample 1
        waitForMotors(2); //TODO
        setIntakeWrist("up");
        sleep(300);
        autoRotate(0, normalPower);
        moveToPosition(0, -9.5, 0.4);
        sleep(300);
        reverseIntake();
        sleep(500);
        setVerticalSlide("basket high", 1); // raise arm for sample 1
        waitForMotors();//?
        stopIntake();
        moveToPosition(0, -18, 0.4);//move slowly
        waitForArm();
        waitForMotors(1);
        setBucketPosition("drop"); // place sample 1
        sleep(600);

        //start sample 2
        moveToPosition(-15, -18, 0.4);
        sleep(200);
        setBucketPosition("carry");
        setVerticalSlide("low", 0.8);
        waitForMotors(2);
        autoRotate(90, 0.4);
        moveToPosition(-18, -25, 0.4);
//        intakeSlide.setPower(0.1);
//        sleep(200);
//        intakeSlide.setPower(0);
        setIntakeWrist("down");
        startIntake();
//        intakeSlide.setPower(-0.1);
//        sleep(200);
//        intakeSlide.setPower(0);
        waitForMotors(1);
        moveToPosition(-24.5, -25, 0.4);
        waitForMotors(1);//TODO
        sleep(150);
        setIntakeWrist("up");
        sleep(300);
        autoRotate(0, normalPower);
        moveToPosition(0, -9.5, normalPower);
        reverseIntake();
        sleep(500);
        setVerticalSlide("basket high", 1); // raise arm sample 2
        stopIntake();
        waitForMotors();
        moveToPosition(0, -18, 0.4);//approach slowly
        waitForMotors(1);
        setBucketPosition("drop"); // place sample 2
        sleep(600);
        moveToPosition(-29.412, -5, 0.6);
        sleep(200);
        setBucketPosition("carry");
        setVerticalSlide("low", verticalSlidePower);
        waitForMotors();
        autoRotate(180, 0.6);
        autoRotate(180, 0.1);
        sleep(100);
        setIntakeWrist("down");
        startIntake();

    }
}
