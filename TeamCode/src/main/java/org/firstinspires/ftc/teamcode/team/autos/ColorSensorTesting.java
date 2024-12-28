package org.firstinspires.ftc.teamcode.team.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.team.DarienOpMode;

@Autonomous
public class ColorSensorTesting extends DarienOpMode {

    // Positions:
    // 0: sample is detectable but out of reach
    // 1: sample is in reach of the bootwheel
    // 2: sample is collected
    public int[] RGB_YELLOW_AT_POS0 = {38, 65, 37};
    public int[] RGB_YELLOW_AT_POS1 = {55, 89, 41};
    public int[] sensorRGB = {0, 0, 0};
    public boolean isItYellow;

    public void runOpMode() {
        initControls();

        waitForStart();

        while (opModeIsActive()) {
            // read the sensor values
            sensorRGB[0] = intakeColorSensor.red();
            sensorRGB[1] = intakeColorSensor.green();
            sensorRGB[2] = intakeColorSensor.blue();
            // pass the sensor values to your isYellow function to test if the sample is yellow or not.
            isItYellow = isYellow(sensorRGB[0], sensorRGB[1], sensorRGB[2]);

            // Display the values on the Driver Station.
            telemetry.addData("R: ", sensorRGB[0]);
            telemetry.addData("G: ", sensorRGB[1]);
            telemetry.addData("B: ", sensorRGB[2]);
            telemetry.addData("Yellow:", isItYellow);
            telemetry.update();

            if (gamepad1.a) {
                // test if sample is yellow

            }

        }
    }

    public String whatColor(int r, int g, int b) {
        // return a string with the name of the color as either "Y", "R" or "B".
        return "";
    }

    public boolean isYellow(int r, int g, int b) {
        // RATIOS
        float r2g = (float) r / g;
        float b2g = (float) b / g;

        return .55 <= r2g && r2g <= .75
                && .15 <= b2g && b2g <= .55;

    }

    /*
    public boolean matchColor(String targetColorAndPosition, int[] rgbValue) {
        switch (targetColorAndPosition) {
            case "Y1":
                // Yellow at position 1 (in reach of bootwheels)
                if (rgbValue[0] >= RGB_YELLOW_AT_POS1[0]
                        && rgbValue[1] >= RGB_YELLOW_AT_POS1[1]
                        && rgbValue[2] >= RGB_YELLOW_AT_POS1[2]) {
                    return true;
                }
                break;
            default:
                // do nothing
        }
        return false;
    }

     */
}