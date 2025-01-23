//package org.firstinspires.ftc.teamcode.team.autos;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//
//import org.firstinspires.ftc.teamcode.team.DarienOpMode;
//
//@Disabled
//@Autonomous
//public class ColorSensorTesting extends DarienOpMode {
//
//    // Positions:
//    // 0: sample is detectable but out of reach
//    // 1: sample is in reach of the bootwheel
//    // 2: sample is collected
//    public int[] RGB_YELLOW_AT_POS0 = {38, 65, 37};
//    public int[] RGB_YELLOW_AT_POS1 = {55, 89, 41};
//    public int[] sensorRGB = {0, 0, 0};
//    public boolean isItYellow;
//    public boolean isItRed;
//    public boolean isItBlue;
//
//    public void runOpMode() {
//        initControls();
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            // read the sensor values
//            sensorRGB[0] = intakeColorSensor.red();
//            sensorRGB[1] = intakeColorSensor.green();
//            sensorRGB[2] = intakeColorSensor.blue();
//            // pass the sensor values to your isYellow, isRed, and isBlue functions to test if the sample is yellow, red, or blue.
//            isItYellow = isYellow(sensorRGB[0], sensorRGB[1], sensorRGB[2]);
//            isItRed = isRed(sensorRGB[0], sensorRGB[1], sensorRGB[2]);
//            isItBlue = isBlue(sensorRGB[0], sensorRGB[1], sensorRGB[2]);
//
//            // Display the values on the Driver Station.
//            telemetry.addData("R: ", sensorRGB[0]);
//            telemetry.addData("G: ", sensorRGB[1]);
//            telemetry.addData("B: ", sensorRGB[2]);
//            telemetry.addData("Yellow:", isItYellow);
//            telemetry.addData("Red:", isItRed);
//            telemetry.addData("Blue:", isItBlue);
//            telemetry.addData("Color: ", whatColor(sensorRGB[0], sensorRGB[1], sensorRGB[2]));
//            telemetry.update();
//
//            if (gamepad1.a) {
//                // test if sample is yellow
//
//            }
//
//        }
//    }
//
//    public String whatColor(int r, int g, int b) {
//        // return a string with the name of the color as either "Y", "R" or "B".
//        String color = "";
//        if (isYellow(r, g, b)) {
//            color = "Y";
//        } else if (isRed(r, g, b)) {
//            color = "R";
//        } else if (isBlue(r, g, b)) {
//            color = "B";
//        }
//        return color;
//    }
//
//    public boolean isYellow(int r, int g, int b) {
//        // RATIOS
//        float r2g = (float) r / g;
//        float b2g = (float) b / g;
//        float r2b = (float) r / b;
//
//        return .55 <= r2g && r2g <= .75
//                && .15 <= b2g && b2g <= .55
//                && 1.00 <= r2b && r2b <= 4.0;
//
//    }
//
//    public boolean isRed(int r, int g, int b) {
//        // RATIOS
//        float r2g = (float) r / g;
//        float b2g = (float) b / g;
//        float r2b = (float) r / b;
//
//        return .65 <= r2g && r2g <= 1.90
//                && .35 <= b2g && b2g <= .70
//                && 0.95 <= r2b && r2b <= 4.85;
//
//    }
//
//    public boolean isBlue(int r, int g, int b) {
//        // RATIOS
//        float r2g = (float) r / g;
//        float b2g = (float) b / g;
//        float r2b = (float) r / b;
//
//        return .35 <= r2g && r2g <= .55
//                && .80 <= b2g && b2g <= 2.30
//                && 0.15 <= r2b && r2b <= .70;
//
//    }
//
//    /*
//    public boolean matchColor(String targetColorAndPosition, int[] rgbValue) {
//        switch (targetColorAndPosition) {
//            case "Y1":
//                // Yellow at position 1 (in reach of bootwheels)
//                if (rgbValue[0] >= RGB_YELLOW_AT_POS1[0]
//                        && rgbValue[1] >= RGB_YELLOW_AT_POS1[1]
//                        && rgbValue[2] >= RGB_YELLOW_AT_POS1[2]) {
//                    return true;
//                }
//                break;
//            default:
//                // do nothing
//        }
//        return false;
//    }
//
//     */
//}