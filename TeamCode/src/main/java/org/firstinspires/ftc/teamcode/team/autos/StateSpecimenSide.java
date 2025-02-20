package org.firstinspires.ftc.teamcode.team.autos;
// JMJ

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.team.DarienOpModeAuto;

@Autonomous(name = "State: Specimen Side (R)", group = "State")
@Disabled
public class StateSpecimenSide extends DarienOpModeAuto {
    @Override
    public void runOpMode() throws InterruptedException {

        initControls();
        setBucketPosition("up");
        setSamplePitch("arm down");
        sampleYaw.setPosition(POS_SAMPLE_YAW_CENTER);
        waitForStart();
        setIntakeSlidePower(-0.05);
        setVerticalSlide("high chamber below", verticalSlidePower);

        // Specimen 1
        moveToPosition(-2, 32, 0.6);
        specimenClaw.setPosition(specimenClawClosed - 0.02);
        setSpecimenWrist("place");
        waitForMotors(1.2);
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
        waitForMotors(0.5);

        moveToPosition(35.5, 23.5, 1); // go to first sample
        sleep(500);
        readyToPickupSample();
        waitForMotors(); // was 3
//        if (Math.abs(getRawHeading()) > 5) {
//            autoRotate(0, 0.2);
//        }
        pickupSample();
        sleep(300);
        moveToPosition(getXPos(), 2, 0.5);
        placeSampleInBucket();
        waitForMotors(2);
        setSampleClaw("open");
        sleep(600);

        //sample 2
        moveToPosition(45, 23.5, 0.6); // go to second sample
        readyToPickupSample();
        waitForMotors();//was 1.2
        pickupSample();
        sleep(100);
        moveToPosition(getXPos(), 0, 0.4);
        placeSampleInBucket();
        waitForMotors(2);
        setSampleClaw("open");
        sleep(500);

        // Specimen 2: Pick up specimen from wall
        setSpecimenClaw("closed"); // keeps the claw slightly open to allow the specimen to fall into position
        sleep(250);
        setVerticalSlide("high chamber below", verticalSlidePower);

        // Start placing specimens on bar
        moveToPosition(-4, 27, 1);
        setSpecimenWrist("place");
        waitForMotors(2.5, true); // was 2.5
        specimenClaw.setPosition(specimenClawClosed - 0.02); // tightens the claw so the specimen doesnt wobble as we clip it
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
        waitForMotors(2.5, true, 2); // was 2
        autoRotate(0, normalPower);
        moveToPosition(getXPos(), -2, 0.3);
        waitForMotors(1); // was 1
        //print("done", "");
        setSpecimenClaw("closed");
        sleep(300);
        setVerticalSlide("high chamber below", verticalSlidePower);


        // Start placing specimens on bar
        moveToPosition(-5, 27, 1);
        setSpecimenWrist("place");
        waitForMotors(2, true);
        specimenClaw.setPosition(specimenClawClosed - 0.02);
        waitForArm();
        moveToPosition(getXPos(), 33, 0.3);
        waitForMotors(0.5);

        setVerticalSlide("high chamber place", verticalSlidePower);
        waitForArm();
        setSpecimenClaw("open");
        setSpecimenWrist("pickup");
        setSamplePitch("arm down");
        setVerticalSlide("low", verticalSlidePower);

        // Specimen 4: Pickup specimen from wall
        moveToPosition(33, 8, 1);
        waitForMotors(2, true, 2);
        autoRotate(0, normalPower);
        moveToPosition(getXPos(), -7, 0.3);
        waitForMotors(0.4);
        print("done", "");
        setSpecimenClaw("closed");
        sleep(300);
        setVerticalSlide("high chamber below", verticalSlidePower);


        // Start placing specimens on bar
        moveToPosition(-7, 27, 1);
        setSpecimenWrist("place");
        waitForMotors(2, true);
        specimenClaw.setPosition(specimenClawClosed - 0.02);
        waitForArm();
        moveToPosition(getXPos(), 34, 0.3);
        waitForMotors(0.5);

        setVerticalSlide("high chamber place", verticalSlidePower);
        waitForArm();
        setSpecimenClaw("open");
        setSpecimenWrist("pickup");
        setSamplePitch("arm down");
        setVerticalSlide("low", verticalSlidePower);

        // PARK: Go to observation zone.
        moveToPosition(40, 5, 1);
        waitForMotors(2, true);

    }
}

