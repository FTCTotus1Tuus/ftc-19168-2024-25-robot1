package org.firstinspires.ftc.teamcode.team.autos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.team.DarienOpModeAuto;


@Config
@Autonomous(name = "State: Sample Side (L)", group = "State Qualifier")
public class StateSampleSide extends DarienOpModeAuto {

    public static double[] xVals = {-2, -21.5, 0, -21, 0, -26, -20, 0, 0, -40};
    public static double[] yVals = {-17.5, -12, -19.5, -12, -18.5, -17, -10, -10, -21, -15};
    public static double[] speedVals = {0.3, 0.5, 0.5, 0.5, 0.5, 0.3, 0.3, 0.3, 0.6, 0.5};


    @Override
    public void runOpMode() throws InterruptedException {

        initControls();
        sampleYaw.setPosition(POS_SAMPLE_YAW_CENTER);
        //start with pre-load sample
        setSamplePitch("arm down");
        waitForStart();
        setIntakeSlidePower(-0.05);
        setSamplePitch("arm down");
        setSampleClaw("closed");
        setIntakeWrist("up");
        setBucketPosition("carry");
        setSpecimenWrist("place");
        setVerticalSlide("basket high", 1);
        sleep(800);
        moveToPosition(xVals[0], yVals[0], speedVals[0]);// approach slowly
        waitForMotors(1.5);
        waitForArm();
        setBucketPosition("drop");
        sleep(850);

        // sample number 1
        moveToPosition(xVals[1], yVals[1], speedVals[1]);
        sleep(200);
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
        setSampleClaw("open");
        sleep(300);
        setVerticalSlide("basket high", 1); // raise arm sample 2
        autoRotate(0, 0.3);

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
        setVerticalSlide("low", 0.8);
        autoRotate(135, 0.3);
        setIntakeWrist("down");
        setSamplePitch("ready");
        sampleYaw.setPosition(POS_SAMPLE_YAW_RIGHT2);
        setSampleClaw("openwide");
        sleep(200);
        pickupSample();
        sleep(300);
        waitForArm();
        placeSampleInBucket();
        sleep(300);
        setSampleClaw("open");
        sleep(300);
        setVerticalSlide("basket high", 1); // raise arm sample 2
        autoRotate(0, 0.3);
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
        setSamplePitch("arm down");
        setVerticalSlide("low", verticalSlidePower);
        waitForMotors();
        autoRotate(165, 0.3);
        setIntakeWrist("down");
        sleep(300);
        setSampleClaw("closed");
        sleep(200);
        placeSampleInBucket();
        moveToPosition(getXPos(), yVals[7], speedVals[7]);
        sleep(300);
        setSampleClaw("open");
        waitForMotors();
        setVerticalSlide("basket high", 1);
        autoRotate(0, 0.3);

        //go to the basket for sample 4
        moveToPosition(xVals[8], yVals[8], speedVals[8]);// approach slowly
        waitForMotors(3);
        waitForArm();
        setBucketPosition("drop");
        sleep(800);

        moveToPosition(xVals[9], yVals[9], speedVals[9]);
        sleep(200);
        setSamplePitch("arm down");
        setVerticalSlide("low", 1);
        waitForMotors();
        waitForArm();


        while (opModeIsActive()) {
        }

    }
}
