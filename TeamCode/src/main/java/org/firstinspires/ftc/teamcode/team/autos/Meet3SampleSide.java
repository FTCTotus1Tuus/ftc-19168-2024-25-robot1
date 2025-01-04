package org.firstinspires.ftc.teamcode.team.autos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.team.ColorSensorFunctions;
import org.firstinspires.ftc.teamcode.team.DarienOpModeAuto;

@Config
@Autonomous
public class Meet3SampleSide extends DarienOpModeAuto {
    public ColorSensorFunctions csf = new ColorSensorFunctions();
    public double normalPower = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        initControls();

        waitForStart();
        // intakeSlide.setPower(-0.1);
        setIntakeWrist("slightly up");
        setBucketPosition("carry");
        setSpecimenWrist("place");
        setVerticalSlide("basket high", 0.8);
        sleep(1800); // TODO
        moveToPosition(-3, -17.5, 0.3);// approach slowly
        waitForArm();
        waitForMotors(1.5);
        setBucketPosition("drop");
        sleep(1000); //TODO
        setBucketPosition("carry");

        // sample number 1
        moveToPosition(-15, -15, normalPower);
        sleep(500);
        setVerticalSlide("low", 0.8);
        waitForMotors(2);
        autoRotate(90, normalPower);
        waitForMotors(0.8);
        intakeSlide.setPower(0.1);
        sleep(200);
        intakeSlide.setPower(0);
        setIntakeWrist("down");
        startIntake();
        // Use the color sensor to determine if we have picked up the sample.
        tryGrabbingSample(80, 0.8);
        intakeSlide.setPower(-0.1);
        sleep(1000);
        intakeSlide.setPower(0);
        moveToPosition(-28.5, -15, 0.1); // pickup sample 1
        waitForMotors(2); //TODO
        setIntakeWrist("up");
        sleep(300);
        autoRotate(0, normalPower);
        moveToPosition(0, -9.5, normalPower);
        sleep(300);
        reverseIntake();
        sleep(500);
        setVerticalSlide("basket high", 1); // raise arm for sample 1
        waitForMotors();//?
        stopIntake();
        moveToPosition(0, -18, 0.3);//move slowly
        waitForArm();
        waitForMotors(2.5);
        setBucketPosition("drop"); // place sample 1
        sleep(1000);

        //start sample 2
        moveToPosition(-15, -18, normalPower);
        setBucketPosition("carry");
        setVerticalSlide("low", 0.8);
        waitForMotors(2);
        autoRotate(90, normalPower);
        moveToPosition(-15, -25, normalPower);
        intakeSlide.setPower(0.1);
        sleep(200);
        intakeSlide.setPower(0);
        setIntakeWrist("down");
        startIntake();
        intakeSlide.setPower(-0.1);
        sleep(200);
        intakeSlide.setPower(0);
        waitForMotors(1);
        moveToPosition(-25, -25, 0.3);
        waitForMotors(2);//TODO
        sleep(150);
        setIntakeWrist("up");
        sleep(400);
        autoRotate(0, normalPower);
        moveToPosition(0, -9.5, normalPower);
        reverseIntake();
        sleep(500);
        setVerticalSlide("basket high", 1); // raise arm sample 2
        stopIntake();
        waitForMotors();
        moveToPosition(0, -18, 0.3);//approach slowly
        waitForMotors();
        setBucketPosition("drop"); // place sample 2
        sleep(800);
        moveToPosition(-30, -5, normalPower);
        setVerticalSlide("low", verticalSlidePower);
        waitForMotors();
        autoRotate(180, normalPower);

    }

    public void tryGrabbingSample(int colorIntensityMin, double timeoutSeconds) {
        boolean looping = true;
        double startTime = getRuntime();
        while (looping) {
            intakeSlide.setPower(.1);
            if (csf.yellowColorIntensity(intakeColorSensor.red(), intakeColorSensor.green(), intakeColorSensor.blue()) >= colorIntensityMin || getRuntime() - startTime > timeoutSeconds) {
                // if the yellow color is strong enough, that indicates we have the sample.
                looping = false;
                stopIntake();
                intakeSlide.setPower(0);
            }
        }
    }
}
