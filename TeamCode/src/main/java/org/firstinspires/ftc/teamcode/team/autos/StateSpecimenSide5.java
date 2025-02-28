package org.firstinspires.ftc.teamcode.team.autos;
// JMJ

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.team.DarienOpModeAuto;

@Autonomous(name = "State: Specimen Side (R)", group = "State")
public class StateSpecimenSide5 extends DarienOpModeAuto {
    @Override
    public void runOpMode() throws InterruptedException {

        bucketUp = 0.925;

        initControls();
        setBucketPosition("up");
        setSamplePitch("arm down");
        sampleYaw.setPosition(POS_SAMPLE_YAW_CENTER);
        waitForStart();
        setIntakeSlidePower(-0.1);
        setVerticalSlide("high chamber below", verticalSlidePower);

        // Specimen 1
        moveToPosition(-15, 31.5, 0.95); //TODO
        specimenClaw.setPosition(specimenClawClosed - 0.03);
        setSpecimenWrist("place");
        waitForMotors(1);
        waitForArm();
        sleep(100);
        setVerticalSlide("high chamber place", verticalSlidePower);
        waitForArm();
        setSpecimenClaw("open");
        setSpecimenWrist("pickup");
        setSamplePitch("arm down");
        setVerticalSlide("low", verticalSlidePower);

        // Pick up one floor samples into observation zone, which will become specimens
        updatePosition();
        moveToPosition(getXPos(), 25, 1); // backup from wall
        waitForMotors(0.5);

        moveToPosition(35.5, 23.5, 1); // go to first sample
        sleep(300);
        readyToPickupSample();
        waitForMotors(); // was 3s
        pickupSample();
        sleep(100);
        updatePosition();
        moveToPosition(getXPos(), 4, 0.9); // Set Y target to a few inches from wall to avoid slamming into the wall as robot skids into position.
        placeSampleInBucket();
        sleep(300); //TODO 400-300
        setSampleClaw("open");
        waitForMotors(1.5, false, 1);
        sleep(150);

        //sample 2
        moveToPosition(45.5, 23.5, 1); // go to second sample
        readyToPickupSample();
        waitForMotors();//was 1.2
        pickupSample();
        sleep(100);
        updatePosition();
        moveToPosition(getXPos(), 4, 0.9); // Set Y target to a few inches from wall to avoid slamming into the wall as robot skids into position.
        placeSampleInBucket();
        sleep(300); //TODO 400-250
        setSampleClaw("open");
        waitForMotors(1.5, false, 1);
        sleep(150);

        //sample 3
        moveToPosition(55, 23.5, 1); // go to third sample

        setIntakeWrist("down");
        setSamplePitch("ready");
        sampleYaw.setPosition(POS_SAMPLE_YAW_CENTER);
        setSampleClaw("open");

        waitForMotors(1.5);
        pickupSample();
        sleep(100);
        moveToPosition(40, -2, 1); // Set a negative Y position to reach the wall.
        placeSampleInBucket();

        boolean looping = true;
        double errorX = 0;
        double errorY = 0;
        double errorXp;
        double errorYp;
        double errorH;
        double errorHrads;

        //PID commands
        double movement_pduty = 0;
        double movement_iduty = 0;
        double movement_power;

        Pose2D velocity;

        double startTime = this.time;


        while (looping) {

            if (this.time - startTime > .3) {
                setSampleClaw("open");
            }

            updatePosition(); // VERY NESSCESSARY WHENEVER WE ARE MOVING

            errorX = targetPosX - getXPos();
            errorY = targetPosY - getYPos();
            errorH = getErrorRot(targetPosH);
            errorHrads = Math.toRadians(errorH) * 7;

            errorXp = (errorX * Math.cos(Math.toRadians(getRawHeading()))) + errorY * Math.sin(Math.toRadians(getRawHeading()));
            errorYp = (-errorX * Math.sin(Math.toRadians(getRawHeading()))) + errorY * Math.cos(Math.toRadians(getRawHeading()));


            {
                if (getHypotenuse(errorXp, errorYp) < distanceToSlowdown) {
                    setPower(slowdownPower, errorXp, errorYp, errorHrads); // add pid?
                    telemetry.addData("final approach - pid", "");
                } else {

                    movement_pduty = clamp(movement_pgain * Math.pow(getHypotenuse(errorXp, errorYp), 3 / 2), -1, 1);
                    movement_iduty = clamp(movement_igain * (getHypotenuse(errorXp, errorYp)) + movement_iduty, -.7, .7);
                    movement_power = clamp(movement_pduty + movement_iduty, -currentMovementPower, currentMovementPower);
                    setPower(movement_power, errorXp, errorYp, errorHrads);
                    telemetry.addData("current move power: ", movement_power);
                }
            }
            //exit controls
            if (getHypotenuse(errorX, errorY) <= 0.5) {
                looping = false;
            } else if (getHypotenuse(odo.getVelX(), odo.getVelY()) <= minimumXYspeed &&
                    getHypotenuse(errorX, errorY) < acceptableXYError * 4) {
                looping = false;
            } else if ((this.time - movementStartTime) > 1.5) {
                looping = false;
            }
        }

        setPower(0, 0, 0, 0);


        // Specimen 2: Pick up specimen from wall
        setSpecimenClaw("closed"); // keeps the claw slightly open to allow the specimen to fall into position
        sleep(200);
        setVerticalSlide("high chamber below", verticalSlidePower);

        // Place specimen on chamber
        moveToPosition(-4, 27, 1); //TODO
        setSpecimenWrist("place");
        waitForMotors(1.8, true, 2, true); // was 2.5
        specimenClaw.setPosition(specimenClawClosed - 0.02); // tightens the claw so the specimen doesnt wobble as we clip it
        setPower(0.35, 0, 1, 0);
        sleep(200);


        setVerticalSlide("high chamber place", verticalSlidePower);
        waitForArm();
        setPower(0, 0, 0, 0);
        setSpecimenClaw("open");
        setSpecimenWrist("pickup");
        setSamplePitch("arm down");
        setVerticalSlide("low", verticalSlidePower);

        // Specimen 3: Pickup specimen from wall
        moveToPosition(33, 3, 1);
        waitForMotors(2.5, true, 2, true); // was 2
        autoRotate(0, normalPower);
        moveToPosition(getXPos(), -2, 0.5);
        waitForMotors(0.3);
        //print("done", "");
        setSpecimenClaw("closed");
        sleep(200);
        setVerticalSlide("high chamber below", verticalSlidePower);

        // Place specimen on chamber
        moveToPosition(-2, 27, 1); //TODO
        setSpecimenWrist("place");
        waitForMotors(1.8, true, 2, true);
        specimenClaw.setPosition(specimenClawClosed - 0.04);

        setPower(0.4, 0, 1, 0);
        sleep(200);


        setVerticalSlide("high chamber place", verticalSlidePower);
        waitForArm();
        setPower(0, 0, 0, 0);


        setSpecimenClaw("open");
        setSpecimenWrist("pickup");
        setSamplePitch("arm down");
        setVerticalSlide("low", verticalSlidePower);

        // Specimen 4: Pickup specimen from wall
        moveToPosition(33, 3, 1);
        waitForMotors(2, true, 2, true);
        autoRotate(0, normalPower);
        updatePosition();
        moveToPosition(getXPos(), -7, 0.5);
        waitForMotors(0.3);
        print("done", "");
        setSpecimenClaw("closed");
        sleep(200);
        setVerticalSlide("high chamber below", verticalSlidePower);


        // Place specimen on chamber
        moveToPosition(0.25, 27, 1); //TODO
        setSpecimenWrist("place");
        waitForMotors(1.8, true, 2, true);
        specimenClaw.setPosition(specimenClawClosed - 0.04);

        setPower(0.4, 0, 1, 0);
        sleep(200);


        setVerticalSlide("high chamber place", verticalSlidePower);
        waitForArm();
        setPower(0, 0, 0, 0);

        setSpecimenClaw("open");
        setSpecimenWrist("pickup");
        setSamplePitch("arm down");
        setVerticalSlide("low", verticalSlidePower);

        // Specimen 5: Pickup specimen from wall
        moveToPosition(33, 3, 1);
        waitForMotors(2, true, 2, true);
        autoRotate(0, normalPower);
        updatePosition();
        moveToPosition(getXPos(), -7, 0.5);
        waitForMotors(0.3);
        print("done", "");
        setSpecimenClaw("closed");
        sleep(200);
        setVerticalSlide("high chamber below", verticalSlidePower);

        // Place specimen on chamber
        moveToPosition(1.5, 27, 1); //TODO
        setSpecimenWrist("place");
        waitForMotors(1.8, true, 2, true);
        specimenClaw.setPosition(specimenClawClosed - 0.04);

        setPower(0.3, 0, 1, 0);
        sleep(150);


        setVerticalSlide("high chamber place", verticalSlidePower);
        waitForArm();
        setPower(0, 0, 0, 0);

        setSpecimenClaw("open");
        setSpecimenWrist("pickup");
        setSamplePitch("arm down");
        setVerticalSlide("low", verticalSlidePower);

        // PARK: Go to observation zone.
        moveToPosition(40, 5, 1);

        double errorX1 = 0;
        double errorY1 = 0;
        double errorXp1;
        double errorYp1;
        double errorH1;
        double errorHrads1;

        while (opModeIsActive()) {
            errorX1 = targetPosX - getXPos();
            errorY1 = targetPosY - getYPos();
            errorH1 = getErrorRot(targetPosH);
            errorHrads1 = Math.toRadians(errorH1) * 7;

            errorXp1 = (errorX1 * Math.cos(Math.toRadians(getRawHeading()))) + errorY1 * Math.sin(Math.toRadians(getRawHeading()));
            errorYp1 = (-errorX1 * Math.sin(Math.toRadians(getRawHeading()))) + errorY1 * Math.cos(Math.toRadians(getRawHeading()));


            setPower(1, errorXp1, errorYp1, errorHrads1); // add pid?
        }

    }
}

