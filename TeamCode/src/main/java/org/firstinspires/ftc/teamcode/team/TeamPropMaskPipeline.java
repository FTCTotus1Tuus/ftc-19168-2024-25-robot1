package org.firstinspires.ftc.teamcode.team;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.CameraControl;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;

@Config
public
class TeamPropMaskPipeline implements VisionProcessor
{
    public static int minHueR = 0, minSaturationR = 40, minValueR = 0 , maxHueR = 10, maxSaturationR = 260, maxValueR = 300,
                      minHueB = 70, minSaturationB = 100, minValueB = 0 , maxHueB = 117, maxSaturationB = 260, maxValueB = 300, frameWidth, frameHeight;
    int minHue, minSaturation, minValue, maxHue, maxSaturation, maxValue;

    double[] lastResults = {1,2,3};
    Rect firstThird, secondThird, thirdThird;
    double avgFirstThird, avgSecondThird, avgThirdThird;
    Mat workingMat1 = new Mat(), workingMat2 = new Mat(), workingMat3 = new Mat();
    Mat mask1 = new Mat(), mask2 = new Mat(), mask3 = new Mat();

    public static double clawPositionL;
    public static double clawPositionR;

    public TeamPropMaskPipeline(boolean isBlue) {
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


    public double[] getLastResults() {
        return lastResults;
    }


    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

//            workingMat1.release();
//            workingMat2.release();
//            workingMat3.release();
//
//            mask1.release();
//            mask2.release();
//            mask3.release();


        // Calculate the width and height of the frame
        frameWidth = frame.width();
        frameHeight = frame.height();


        // Define the region of interest (ROI) for each third of the screen
        firstThird = new Rect(0, 0, frameWidth / 3, frameHeight);
        secondThird = new Rect(frameWidth / 3, 0, frameWidth / 3, frameHeight);
        thirdThird = new Rect(2 * (frameWidth / 3), 0, frameWidth / 3, frameHeight);

        workingMat1 = frame.clone();


        // Apply a color mask to the workingMat (for example, to highlight a specific color)


        Imgproc.cvtColor(workingMat1, workingMat2, Imgproc.COLOR_RGB2HSV);
        Core.inRange(workingMat2, new Scalar(minHue, minSaturation, minValue), new Scalar(maxHue, maxSaturation, maxValue), workingMat3);

        mask1 = workingMat3.submat(firstThird);
        mask2 = workingMat3.submat(secondThird);
        mask3 = workingMat3.submat(thirdThird);


        avgFirstThird = Core.countNonZero(mask1);
        avgSecondThird = Core.countNonZero(mask2);
        avgThirdThird = Core.countNonZero(mask3);


        lastResults[0] = avgFirstThird;
        lastResults[1] = avgSecondThird;
        lastResults[2] = avgThirdThird;


        workingMat1.release();
        workingMat2.release();
        workingMat3.release();

        mask1.release();
        mask2.release();
        mask3.release();


        return frame;
//            return workingMat3.adjustROI(0,0,frameWidth,frameHeight);
//        }
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

}}