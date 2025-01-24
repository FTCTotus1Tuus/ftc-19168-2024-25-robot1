package org.firstinspires.ftc.teamcode.team.autos;
// JMJ

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.team.DarienOpModeAuto;

@Autonomous(name = "Qual: Specimen Side", group = "State Qualifier")
public class QualifierSpecimenSide extends DarienOpModeAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        SparkFunOTOS.Pose2D currentPosition;

        initControls();
        waitForStart();
        setBucketPosition("carry");
        setVerticalSlide("high chamber below", verticalSlidePower);

        // Specimen 1
        moveToPosition(-2, 32, 0.6);
        setSpecimenClaw("closed");
        setSpecimenWrist("place");
        waitForMotors(1);
        waitForArm();

        setVerticalSlide("high chamber place", verticalSlidePower);
        waitForArm();
        setSpecimenClaw("open");
        setSpecimenWrist("pickup");
        setVerticalSlide("low", verticalSlidePower);

        //reset odometry angle (has tendency to drift)
        currentPosition = new SparkFunOTOS.Pose2D(getXPos(), getYPos(), 0);
        myOtos.setPosition(currentPosition);

        // Pick up one floor samples into observation zone, which will become specimens
        moveToPosition(getXPos(), 25, 1); // backup from wall
        waitForMotors(0.25);
        moveToPosition(34, 15, 0.8); // go to first sample
        setIntakeSlidePower(-0.1);
        setIntakeWrist("down");
        waitForMotors(2); // possibly bad idea was 3
        moveToPosition(getXPos(), 30, 0.25); // 0.27 is an important number 0.2 may be importanter
        waitForMotors(1.7);
        setIntakeWrist("up");
        moveToPosition(getXPos(), 9, 0.7);
        sleep(500); // allows time for intake wrist to go fully up
        reverseIntake(-0.25);
        waitForMotors(1);
        setIntakeSlidePower(0);
        setBucketPosition("drop"); // drop first sample
        sleep(500); // allow time for bucket to fully go to position

        //sample 2
        moveToPosition(45, 15, 0.8); // go to second sample
        setIntakeWrist("down");
        waitForMotors(2);
        setIntakeSlidePower(-0.1);
        setBucketPosition("carry");
        moveToPosition(getXPos(), 30, 0.25);
        waitForMotors(1.5);
        setIntakeWrist("up");
        moveToPosition(getXPos(), 7, 0.6);
        sleep(500); // allows time for intake wrist to go fully up
        reverseIntake(-0.25);
        waitForMotors(1);
        setIntakeSlidePower(0);
        // rotate to dump sample in corner
        setBucketPosition("drop"); // drop second sample
        autoRotate(35, 0.8);
        sleep(500); // allow time for bucket to fully go to position
        autoRotate(0, 0.8);

        // Specimen 2: Pick up specimen from wall
        moveToPosition(getXPos(), -7, 0.4);
        waitForMotors(0.75);
        //print("done", "");
        setSpecimenClaw("closed");
        sleep(250);
        setSpecimenWrist("place");

        // Start placing specimens on bar
        moveToPosition(-5, 27, 1);
        setSpecimenClaw("closed");
        setSpecimenWrist("place");
        setVerticalSlide("high chamber below", verticalSlidePower);
        waitForMotors(2.5);
        waitForArm();
        moveToPosition(-5, 34, 0.25);
        waitForMotors(1);


        setVerticalSlide("high chamber place", verticalSlidePower);
        waitForArm();
        setSpecimenClaw("open");
        setSpecimenWrist("pickup");
        setVerticalSlide("low", verticalSlidePower);

//        //reset odometry angle (has tendency to drift), assuming we are flush against the sub wall
//        currentPosition = new SparkFunOTOS.Pose2D(getXPos(), getYPos(), 0);
//        myOtos.setPosition(currentPosition);


        // Specimen 3: Pickup specimen from wall
        moveToPosition(33, 5, 1);
        waitForMotors(2);
        autoRotate(0, normalPower);
        moveToPosition(getXPos(), -7, 0.3);
        waitForMotors(1);
        //print("done", "");
        setSpecimenClaw("closed");
        sleep(300);
        setSpecimenWrist("place");

        // Start placing specimens on bar
        moveToPosition(-8, 27, 1);
        setVerticalSlide("high chamber below", verticalSlidePower);
        waitForMotors(2.5);
        waitForArm();
        moveToPosition(getXPos(), 32, 0.3);
        waitForMotors(1);

        setVerticalSlide("high chamber place", verticalSlidePower);
        waitForArm();
        setSpecimenClaw("open");
        setSpecimenWrist("pickup");
        setVerticalSlide("low", verticalSlidePower);

//        // Specimen 4: Pickup specimen from wall
//        moveToPosition(33, 8, 1);
//        waitForMotors(2);
//        autoRotate(0, normalPower);
//        moveToPosition(getXPos(), -7, 0.3);
//        waitForMotors(1);
//        print("done", "");
//        setSpecimenClaw("closed");
//        sleep(300);
//        setSpecimenWrist("place");
//
//        // Start placing specimens on bar
//        moveToPosition(-10, 27, 1);
//        setVerticalSlide("high chamber below", verticalSlidePower);
//        waitForMotors(2.5);
//        waitForArm();
//        moveToPosition(getXPos(), 33, 0.3);
//        waitForMotors(1);
//
//        setVerticalSlide("high chamber place", verticalSlidePower);
//        waitForArm();
//        setSpecimenClaw("open");
//        setSpecimenWrist("pickup");
//        setVerticalSlide("low", verticalSlidePower);

        // PARK: Go to observation zone.
        moveToPosition(40, 5, 1);
        waitForMotors(2);

    }
}

