package org.firstinspires.ftc.teamcode.team.autos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.team.DarienOpModeAuto;


@Config

@Autonomous(name = "Qual: Sample Side", group = "State Qualifier")
public class QualSampleSide extends DarienOpModeAuto {
    public double normalPower = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        initControls();

        waitForStart();
        setIntakeWrist("slightly up");
        setBucketPosition("carry");
        setSpecimenWrist("place");
        setVerticalSlide("basket high", 1);
        sleep(600); // TODO
        moveToPosition(-3, -17.5, 0.3);// approach slowly
        waitForArm();
        waitForMotors(0.5);
        setBucketPosition("drop");

        // sample number 1
        moveToPosition(-14, -13.65, 0.4);
        sleep(200);
        setSampleClaw("closed");
        setBucketPosition("carry");
        setVerticalSlide("low", 0.8);
        waitForMotors(2);
        readyToPickupSample();
        autoRotate(90, 0.4);
        pickupSample();
        sleep(300);
        placeSampleInBucket();
        moveToPosition(-20, getYPos(), 0.4);
        waitForMotors(1);

        //go to basket for sample 1
        autoRotate(45, normalPower);
        setVerticalSlide("basket high", 1); // raise arm sample 2
        moveToPosition(-3, -22, 0.4);//approach slowly
        waitForMotors(1.5);
        waitForArm();
        setBucketPosition("drop");
        //start sample 2
        moveToPosition(-15, -18, 0.4);
        setSampleClaw("closed");
        sleep(200);
        setBucketPosition("carry");
        waitForMotors(2);
        readyToPickupSample();
        setVerticalSlide("low", 0.8);
        autoRotate(90, 0.4);
        moveToPosition(-18, -26.5, 0.4);
        pickupSample();
        sleep(300);
        placeSampleInBucket();
        moveToPosition(-15, getYPos(), normalPower);
        waitForMotors(1);

        //go to basket for sample 2
        autoRotate(45, normalPower);
        setVerticalSlide("basket high", 1); // raise arm sample 2
        moveToPosition(0, -23, 0.5);//approach slowly
        waitForMotors(1.5);
        waitForArm();
        setBucketPosition("drop");
        sleep(600);
        setSampleClaw("closed");

        //go to third sample
        moveToPosition(-31, -5, 0.6);
        sleep(200);
        setBucketPosition("carry");
        waitForMotors();
        setVerticalSlide("low", verticalSlidePower);
        autoRotate(180, 0.6);
        autoRotate(180, 0.1);
        readyToPickupSample();
        moveToPosition(getXPos(), -15, 0.3);
        pickupSample();
        moveToPosition(-1, -22, 0.4);
        waitForMotors();
        while (opModeIsActive()) {
        }

    }
}
