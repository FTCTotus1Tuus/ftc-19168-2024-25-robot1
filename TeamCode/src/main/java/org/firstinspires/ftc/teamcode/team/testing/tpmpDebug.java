package org.firstinspires.ftc.teamcode.team.testing;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;

@Config
class tpmpDebug extends OpenCvPipeline
{
    public static int minHueR = 0, minSaturationR = 40, minValueR = 0 , maxHueR = 10, maxSaturationR = 260, maxValueR = 300,
            minHueB = 70, minSaturationB = 100, minValueB = 0 , maxHueB = 117, maxSaturationB = 260, maxValueB = 300,

    lastResults = 1, frameWidth, frameHeight;
    int minHue, minSaturation, minValue, maxHue, maxSaturation, maxValue;

    Rect firstThird, secondThird, thirdThird;
    double avgFirstThird, avgSecondThird, avgThirdThird;
    Mat workingMat1 = new Mat(), workingMat2 = new Mat(), workingMat3 = new Mat();
    Mat mask1 = new Mat(), mask2 = new Mat(), mask3 = new Mat();

    public tpmpDebug(boolean isBlue) {
        // true = blue false = red
        setColour(isBlue);
    }
    public void setColour(boolean isBlue) {
        // true = blue false = red
        if (isBlue) {
            minHue = minHueB;
            minSaturation = minSaturationB;
            minValue = minValueB;
            maxHue = maxHueB;
            maxSaturation = maxSaturationB;
            maxValue = maxValueB;
        } else {
            minHue = minHueR;
            minSaturation = minSaturationR;
            minValue = minValueR;
            maxHue = maxHueR;
            maxSaturation = maxSaturationR;
            maxValue = maxValueR;
        }
    }
    @Override
    public Mat processFrame(Mat input) {

        workingMat1.release();
        workingMat2.release();
        workingMat3.release();

        mask1.release();
        mask2.release();
        mask3.release();


        // Calculate the width and height of the frame
        frameWidth = input.width();
        frameHeight = input.height();


        // Define the region of interest (ROI) for each third of the screen
        firstThird = new Rect(0, 0, frameWidth / 3, frameHeight);
        secondThird = new Rect(frameWidth / 3, 0, frameWidth / 3, frameHeight);
        thirdThird = new Rect(2 * (frameWidth / 3), 0, frameWidth / 3, frameHeight);

        workingMat1 = input.clone();


        // Apply a color mask to the workingMat (for example, to highlight a specific color)


        Imgproc.cvtColor(workingMat1, workingMat2, Imgproc.COLOR_RGB2HSV);
        Core.inRange(workingMat2, new Scalar(minHue, minSaturation, minValue), new Scalar(maxHue, maxSaturation, maxValue), workingMat3);

        mask1 = workingMat3.submat(firstThird);
        mask2 = workingMat3.submat(secondThird);
        mask3 = workingMat3.submat(thirdThird);


        avgFirstThird = Core.countNonZero(mask1);
        avgSecondThird = Core.countNonZero(mask2);
        avgThirdThird = Core.countNonZero(mask3);


//             You can now use the average values to determine the object's position
        if (avgFirstThird > avgSecondThird && avgFirstThird > avgThirdThird) {lastResults = 1;}
        else if (avgSecondThird > avgFirstThird && avgSecondThird > avgThirdThird) {lastResults = 2;}
        else { lastResults = 3;}


//            if (maskSel == 2) {
//                return workingMat1;
//            } else if (maskSel == 1) {
//                return mask1;
//            }else {
        return workingMat3.adjustROI(0,0,frameWidth,frameHeight);
//        }
    }

    public int getLastResults() {
        return lastResults;
    }
}