package org.firstinspires.ftc.teamcode.team.testing;

import android.graphics.Canvas;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
@Disabled
public
class ypppDebug extends OpenCvPipeline {
    public static int minHue = 0, minSaturation = 60, minValue = 200, maxHue = 256, maxSaturation = 260, maxValue = 256;

    int frameWidth, frameHeight;

    Rect leftHalf, rightHalf;
    Mat workingMat1 = new Mat(), workingMat2 = new Mat(), workingMat3 = new Mat();
    Mat mask1 = new Mat(), mask2 = new Mat();

    int leftCount, rightCount;

    boolean lastResults;


    public ypppDebug() {
    }


    public boolean getLastResults() {
        return lastResults;
    }


    @Override
    public Mat processFrame(Mat frame) {
        workingMat3.release();        // Calculate the width and height of the frame
        frameWidth = frame.width();
        frameHeight = frame.height();

        // Define the region of interest (ROI) for each third of the screen
        leftHalf = new Rect(0, 0, frameWidth / 3, frameHeight);
        rightHalf = new Rect(frameWidth / 3, 0, frameWidth / 3, frameHeight);

        workingMat1 = frame.clone();
        // Apply a color mask to the workingMat
        Imgproc.cvtColor(workingMat1, workingMat2, Imgproc.COLOR_RGB2HSV);
        Core.inRange(workingMat2, new Scalar(minHue, minSaturation, minValue), new Scalar(maxHue, maxSaturation, maxValue), workingMat3);

        mask1 = workingMat3.submat(leftHalf);
        mask2 = workingMat3.submat(rightHalf);

        leftCount = Core.countNonZero(mask1);
        rightCount = Core.countNonZero(mask2);


        lastResults = (leftCount > rightCount); // true is left false is right

        workingMat1.release();
        workingMat2.release();

        mask1.release();
        mask2.release();


//        return frame;
        return workingMat3.adjustROI(0, 0, frameWidth, frameHeight);
//        }
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}