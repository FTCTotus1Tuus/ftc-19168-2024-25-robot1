package org.firstinspires.ftc.teamcode.team.autos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.team.DarienOpModeAuto;


@Config
@Autonomous(name = "State: Sample Side (L)", group = "State Qualifier")
public class StateSampleSide extends DarienOpModeAuto {

    public static double[] xVals = {-2, -21.5, 0, -22.5, 0, -27.5, -20, 0, 0, -40};
    public static double[] yVals = {-17.5, -12.5, -19.5, -11.5, -18.5, -15.75, -10, -10, -19, -15};
    public static double[] speedVals = {0.325, 1, 0.5, 1, 0.5, 1, 0.3, 0.3, 0.3, 0.3};


    @Override
    public void runOpMode() throws InterruptedException {

        initControls();
        sampleYaw.setPosition(POS_SAMPLE_YAW_CENTER);
        //start with pre-load sample
        setSamplePitch("arm down");
        waitForStart();
        setIntakeSlidePower(-0.15);
        setSamplePitch("arm down");
        setSampleClaw("closed");
        setIntakeWrist("up");
        setBucketPosition("carry");
        setSpecimenWrist("place");
        setVerticalSlide("basket high", 1);
        sleep(800);
        moveToPosition(xVals[0], yVals[0], speedVals[0]);// approach slowly
        waitForMotors(1.25);
        waitForArm();
        setBucketPosition("drop");
        sleep(850);

        // sample number 1
        moveToPosition(xVals[1], yVals[1], speedVals[1]);
        sleep(200);
        setBucketPosition("carry");
        setSamplePitch("arm down");
        setVerticalSlide("low", 1);
        waitForMotors(2);
        readyToPickupSample();
        autoRotate(90, 0.6);
        pickupSample();
        sleep(300);
        waitForArm();
        placeSampleInBucket();
        sleep(350);
        setSampleClaw("open");
        sleep(300);
        setVerticalSlide("basket high", 1); // raise arm sample 2
        autoRotate(0, 0.6);
        sleep(100);

        //go to basket for sample 1
        moveToPosition(xVals[2], yVals[2], speedVals[2]);// approach slowly

        waitForMotors(3);
        waitForArm();
        setBucketPosition("drop");
        sleep(800);

        //start sample 2
        moveToPosition(xVals[3], yVals[3], speedVals[3]);
        setSampleClaw("closed");
        sleep(200);
        setBucketPosition("carry");
        waitForMotors();
        setSamplePitch("arm down");
        setVerticalSlide("low", 1);
        setIntakeWrist("down");
        setSamplePitch("ready");
        sampleYaw.setPosition(POS_SAMPLE_YAW_RIGHT2);
        setSampleClaw("openwide");
        autoRotate(135, 0.6);
        pickupSample();
        sleep(300);
        placeSampleInBucket();
        sleep(350);
        waitForArm();
        setSampleClaw("open");
        sleep(300);
        setVerticalSlide("basket high", 1); // raise arm sample 2
        autoRotate(0, 0.6);
        //go to basket for sample 2
        moveToPosition(xVals[4], yVals[4], speedVals[4]);// approach slowly
        waitForMotors(3);
        waitForArm();
        setBucketPosition("drop");
        sleep(800);

        //start sample 4
        moveToPosition(xVals[5], yVals[5], speedVals[5]);
        setBucketPosition("carry");
        setSamplePitch("arm down");
        sleep(300);
        intakeWrist.setPosition(0.55);
        setSamplePitch("ground");
        sampleYaw.setPosition(POS_SAMPLE_YAW_RIGHT_MAX);
        setSampleClaw("open");
        setVerticalSlide("low", verticalSlidePower);
        waitForMotors();
        autoRotate(165, 0.6);
        setIntakeWrist("down");
        sleep(400);
        setSampleClaw("closed");
        sleep(200);
        placeSampleInBucket();
        autoRotate(0, 0.6);
        setSampleClaw("open");
        sleep(300);
        setVerticalSlide("basket high", 1);

        //go to the basket for sample 4
        moveToPosition(xVals[8], yVals[8], speedVals[8]);// approach slowly
        waitForMotors(5);
        waitForArm();
        setBucketPosition("drop");
        sleep(800);

        moveToPosition(xVals[9], yVals[9], speedVals[9]);
        sleep(400);
        setBucketPosition("carry");
        setSamplePitch("arm down");
        setVerticalSlide("low", 1);
        waitForMotors();


        while (opModeIsActive()) {
        }

    }
}
