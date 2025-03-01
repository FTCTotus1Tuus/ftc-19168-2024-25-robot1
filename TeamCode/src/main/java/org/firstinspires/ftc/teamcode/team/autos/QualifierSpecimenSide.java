package org.firstinspires.ftc.teamcode.team.autos;
// JMJ

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.team.DarienOpModeAuto;

@Disabled
@Autonomous(name = "Qual: Specimen Side (R)", group = "State Qualifier")
public class QualifierSpecimenSide extends DarienOpModeAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        // current version test:
        //1: full success


        SparkFunOTOS.Pose2D currentPosition;

        initControls();
        setSamplePitch("arm down");
        sampleYaw.setPosition(POS_SAMPLE_YAW_CENTER);
        waitForStart();
        setIntakeSlidePower(-0.05);
        setBucketPosition("drop");
        setVerticalSlide("high chamber below", verticalSlidePower);

        // Specimen 1
        moveToPosition(-2, 33, 0.6);
        setSpecimenClaw("closed");
        setSpecimenWrist("place");
        waitForMotors(1);
        waitForArm();

        setVerticalSlide("high chamber place", verticalSlidePower);
        waitForArm();
        setSpecimenClaw("open");
        setSpecimenWrist("pickup");
        setSamplePitch("arm down");
        // TODO: Fix so that the bucket on the vertical arm doesn't come crashing down on the sample claw.
        setVerticalSlide("low", verticalSlidePower);

//        //reset odometry angle (has tendency to drift)
//        currentPosition = new SparkFunOTOS.Pose2D(getXPos(), getYPos(), 0);
//        myOtos.setPosition(currentPosition);

        // Pick up one floor samples into observation zone, which will become specimens
        moveToPosition(getXPos(), 25, 1); // backup from wall
        waitForMotors(0.25);

        setBucketPosition("carry");
        moveToPosition(35, 19, 1); // go to first sample
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
        moveToPosition(45, 18, 0.6); // go to second sample
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
        specimenClaw.setPosition(specimenClawClosed + 0.02); // keeps the claw slightly open to allow the specimen to fall into position
        sleep(250);
        setSpecimenWrist("place");

        // Start placing specimens on bar
        moveToPosition(-4, 27, 1);
        setSpecimenWrist("place");
        setVerticalSlide("high chamber below", verticalSlidePower);
        waitForMotors(2.5);
        setSpecimenClaw("closed"); // tightens the claw so the specimen doesnt wobble as we clip it
        waitForArm();
        moveToPosition(getXPos(), 34, 0.25);
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
        specimenClaw.setPosition(specimenClawClosed + 0.02);
        sleep(300);
        setSpecimenWrist("place");

        // Start placing specimens on bar
        moveToPosition(-5, 27, 1);
        setVerticalSlide("high chamber below", verticalSlidePower);
        waitForMotors(2);
        setSpecimenClaw("closed");
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
        specimenClaw.setPosition(specimenClawClosed + 0.02);
        sleep(300);
        setSpecimenWrist("place");

        // Start placing specimens on bar
        moveToPosition(-7, 27, 1);
        setVerticalSlide("high chamber below", verticalSlidePower);
        waitForMotors(2);
        setSpecimenClaw("closed");
        waitForArm();
        moveToPosition(getXPos(), 33, 0.3);
        waitForMotors(0.25);

        setVerticalSlide("high chamber place", verticalSlidePower);
        waitForArm();
        setSpecimenClaw("open");
        setSpecimenWrist("pickup");
        setSamplePitch("arm down");
        setVerticalSlide("low", verticalSlidePower);

        // PARK: Go to observation zone.
        moveToPosition(40, 5, 1);
        waitForMotors(2);

    }
}

