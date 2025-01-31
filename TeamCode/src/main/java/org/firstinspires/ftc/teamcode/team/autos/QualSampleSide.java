package org.firstinspires.ftc.teamcode.team.autos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.team.DarienOpModeAuto;


@Config

@Autonomous(name = "Qual: Sample Side (L)", group = "State Qualifier")
public class QualSampleSide extends DarienOpModeAuto {
    public double normalPower = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        bucketPickup = 0.92;
        bucketPlace = 0.78;
//        sampleClawClosed = 0.70;

        initControls();
        sampleYaw.setPosition(POS_SAMPLE_YAW_CENTER);
        //start with pre-load sample
        setSamplePitch("arm down");
        waitForStart();
        setIntakeSlidePower(-0.05);
        setSamplePitch("arm down");
        setIntakeWrist("up");
        setBucketPosition("carry");
        setSpecimenWrist("place");
        setVerticalSlide("basket high", 1);
        sleep(800);
        moveToPosition(-2, -17.5, 0.3);// approach slowly
        waitForMotors(1.5);
        waitForArm();
        setBucketPosition("drop");
        sleep(800);

        // sample number 1
        moveToPosition(-18, -12, 0.5);
        sleep(200);
        setSampleClaw("closed");
        setBucketPosition("carry");
        setSamplePitch("arm down");
        setVerticalSlide("low", 0.8);
        waitForMotors(2);
        autoRotate(90, 0.3);
        readyToPickupSample();
        sleep(200);
        pickupSample();
        sleep(300);
        waitForArm();
        placeSampleInBucket();
        sleep(300);
        setSamplePitch("arm down");
        setVerticalSlide("basket high", 1); // raise arm sample 2
        autoRotate(0, 0.3);

        //go to basket for sample 1
        moveToPosition(0, -21, 0.5);// approach slowly

        waitForMotors(3);
        waitForArm();
        setBucketPosition("drop");
        sleep(800);

        //start sample 2
        moveToPosition(-17.5, -21, 0.5);
        setSampleClaw("closed");
        sleep(200);
        setBucketPosition("carry");
        waitForMotors();
        setVerticalSlide("low", 0.8);
        autoRotate(90, 0.3);
        readyToPickupSample();
        sleep(200);
        pickupSample();
        sleep(300);
        waitForArm();
        placeSampleInBucket();
        sleep(300);
        setSamplePitch("arm down");
        setVerticalSlide("basket high", 1); // raise arm sample 2
        autoRotate(0, 0.3);
        //go to basket for sample 2
        moveToPosition(0, -18.5, 0.5);// approach slowly
        waitForMotors(3);
        waitForArm();
        setBucketPosition("drop");
        sleep(800);

        //go to park pos
        moveToPosition(-50, -10, 0.3);
        setBucketPosition("carry");
        setSamplePitch("arm down");
        sleep(300);
        setVerticalSlide("low", verticalSlidePower);
        waitForMotors();
        autoRotate(180, 0.3);


//        setBucketPosition("carry");
//        waitForMotors(3);
//        setSamplePitch("arm down");
//        setVerticalSlide("low", verticalSlidePower);
//        autoRotate(270, 0.6);
//        moveToPosition(-52, -25, 270, 0.4);
//
//        setIntakeWrist("down");
//        setSamplePitch("ready");
//        sampleYaw.setPosition(POS_SAMPLE_YAW_CENTER);
//        setSampleClaw("open"); // opens normally as opposed to "openwide" for safety
//
//        waitForMotors(2);
//        pickupSample();
//        sleep(300);
//        placeSampleInBucket();
//        sleep(200);
//        setSamplePitch("arm down");
//        setVerticalSlide("basket high", 1); // raise arm sample 2
//        autoRotate(0, 0.3);
//        moveToPosition(0, -18.5, 0.5);// approach slowly
//
//        //start to go to basket to place sample 4
//        waitForMotors(4);
//        waitForArm();
//        setBucketPosition("drop");
//        sleep(800);
//
//        moveToPosition(-20, 0, 0.4);
//        sleep(200);
//        setSamplePitch("arm down");
//        setVerticalSlide("low", verticalSlidePower);
//        waitForMotors();

        while (opModeIsActive()) {
        }

    }
}
