package org.firstinspires.ftc.teamcode.team.autos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.team.DarienOpModeAuto;


@Config
@Disabled
@Autonomous(name = "Meet 3: Sample Side", group = "Meet 3")
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
        sleep(600); // TODO
        moveToPosition(-3, -17.5, 0.3);// approach slowly
        waitForArm();
        waitForMotors(0.5);
        setBucketPosition("drop");
        setIntakeWrist("down");
        sleep(600); //TODO

        // sample number 1
        moveToPosition(-14, -13.65, 0.4);
        sleep(200);
        setBucketPosition("carry");
        setVerticalSlide("low", 0.8);
        waitForMotors(2);
        autoRotate(90, 0.4);
        startIntake();
        moveToPosition(-26, getYPos(), 0.27); // pickup sample 1
        waitForMotors(1.5); //TODO
        stopIntake();
        setIntakeWrist("up");
        moveToPosition(-20, getYPos(), 0.4);
        sleep(650);
        reverseIntake();
        waitForMotors(0.1);

        //go to basket for sample 1
        autoRotate(45, normalPower);
        setVerticalSlide("basket high", 1); // raise arm sample 2
        stopIntake();
        moveToPosition(-3, -22, 0.4);//approach slowly
        waitForMotors(1.5);
        waitForArm();
        setBucketPosition("drop");
        setIntakeWrist("down");
        sleep(600);

        //start sample 2
        moveToPosition(-15, -18, 0.4);
        sleep(200);
        setBucketPosition("carry");
        waitForMotors(2);
        setVerticalSlide("low", 0.8);
        autoRotate(90, 0.4);
        moveToPosition(-18, -26.5, 0.4);
        startIntake();
        waitForMotors(1);
        moveToPosition(-25.5, getYPos(), 0.27); // go forward to pickup
        waitForMotors(1);//TODO
        sleep(150);
        stopIntake();
        setIntakeWrist("up");
        moveToPosition(-15, getYPos(), normalPower);
        sleep(700);
        reverseIntake();
        waitForMotors(0.25);

        //go to basket for sample 2
        autoRotate(45, normalPower);
        setVerticalSlide("basket high", 1); // raise arm sample 2
        stopIntake();
        moveToPosition(0, -23, 0.5);//approach slowly
        waitForMotors(1.5);
        waitForArm();
        setBucketPosition("drop");
        sleep(600);

        //go to third sample
        moveToPosition(-31, -5, 0.6);
        sleep(200);
        setBucketPosition("carry");
        waitForMotors();
        setVerticalSlide("low", verticalSlidePower);
        autoRotate(180, 0.6);
        autoRotate(180, 0.1);
        sleep(100);
        setIntakeWrist("down");
        startIntake();
        moveToPosition(getXPos(), -15, 0.3);
        waitForMotors();
        setIntakeWrist("up");
        sleep(500);
        reverseIntake();
        moveToPosition(-1, -22, 0.4);
        waitForMotors();
        while (opModeIsActive()) {
        }

    }
}
