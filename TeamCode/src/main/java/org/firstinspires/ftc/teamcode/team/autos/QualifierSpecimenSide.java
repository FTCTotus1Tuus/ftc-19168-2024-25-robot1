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
        setSamplePitch("arm down");
        waitForStart();
        setBucketPosition("drop");
        setVerticalSlide("high chamber below", verticalSlidePower);

        // Specimen 1
        moveToPosition(-2, 33, 0.7);
        setSpecimenClaw("closed");
        setSpecimenWrist("place");
        waitForMotors(1);
        waitForArm();

        setVerticalSlide("high chamber place", verticalSlidePower);
        waitForArm();
        setSpecimenClaw("open");
        setSpecimenWrist("pickup");
        setSamplePitch("arm down");
        setVerticalSlide("low", verticalSlidePower);

//        //reset odometry angle (has tendency to drift)
//        currentPosition = new SparkFunOTOS.Pose2D(getXPos(), getYPos(), 0);
//        myOtos.setPosition(currentPosition);

        // Pick up one floor samples into observation zone, which will become specimens
        moveToPosition(getXPos(), 25, 1); // backup from wall
        waitForMotors(0.25);

        setBucketPosition("carry");
        moveToPosition(35, 18, 1); // go to first sample
        sleep(500);
        readyToPickupSample();
        waitForMotors(3);
        if (Math.abs(getRawHeading()) > 5) {
            autoRotate(0, 0.2);
        }
        pickupSample();
        sleep(300);
        moveToPosition(getXPos(), 11, 0.5);
        placeSampleInBucket();
        waitForMotors(1);
        setBucketPosition("drop"); // drop first sample
        sleep(500); // allow time for bucket to fully go to position

        //sample 2
        moveToPosition(45, 17.5, 0.6); // go to second sample
        setBucketPosition("carry");
        readyToPickupSample();
        waitForMotors(1.2);
        pickupSample();
        sleep(100);
        moveToPosition(getXPos() + 8, 9, 0.3);
        placeSampleInBucket();
        waitForMotors(1);
        // rotate to dump sample in corner
        setBucketPosition("drop"); // drop second sample
        sleep(400); // allow time for bucket to fully go to position
        setSampleClaw("closed");

        // Specimen 2: Pick up specimen from wall
        moveToPosition(33, 9, 0.6);
        waitForMotors(1);
        moveToPosition(getXPos(), -7, 0.4);
        waitForMotors(0.75);
//        setSpecimenClaw("closed");
        specimenClaw.setPosition(specimenClawClosed + 0.01);
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
        waitForMotors(0.5);


        setVerticalSlide("high chamber place", verticalSlidePower);
        waitForArm();
        setSpecimenClaw("open");
        setSpecimenWrist("pickup");
        setSamplePitch("arm down");
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
        specimenClaw.setPosition(specimenClawClosed + 0.01);
        sleep(300);
        setSpecimenWrist("place");

        // Start placing specimens on bar
        moveToPosition(-8, 27, 1);
        setVerticalSlide("high chamber below", verticalSlidePower);
        waitForMotors(2);
        waitForArm();
        moveToPosition(getXPos(), 32, 0.3);
        waitForMotors(0.25);

        setVerticalSlide("high chamber place", verticalSlidePower);
        waitForArm();
        setSpecimenClaw("open");
        setSpecimenWrist("pickup");
        setSamplePitch("arm down");
        setVerticalSlide("low", verticalSlidePower);

        // Specimen 4: Pickup specimen from wall
        moveToPosition(33, 8, 1);
        waitForMotors(2);
        autoRotate(0, normalPower);
        moveToPosition(getXPos(), -7, 0.3);
        waitForMotors(1);
        print("done", "");
        specimenClaw.setPosition(specimenClawClosed + 0.01);
        sleep(300);
        setSpecimenWrist("place");

        // Start placing specimens on bar
        moveToPosition(-10, 27, 1);
        setVerticalSlide("high chamber below", verticalSlidePower);
        waitForMotors(2);
        waitForArm();
        moveToPosition(getXPos(), 33, 0.3);
        waitForMotors(0.25);

        setVerticalSlide("high chamber place", verticalSlidePower);
        waitForArm();
        setSpecimenClaw("open");
        setSpecimenWrist("pickup");
        setVerticalSlide("low", verticalSlidePower);

        // PARK: Go to observation zone.
        moveToPosition(40, 5, 1);
        waitForMotors(2);

    }
}

