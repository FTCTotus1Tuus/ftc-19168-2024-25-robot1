//package org.firstinspires.ftc.teamcode.team;
//
//import android.graphics.Canvas;
//
//import com.acmerobotics.dashboard.config.Config;
//
//import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
//import org.firstinspires.ftc.vision.VisionProcessor;
//import org.opencv.core.Core;
//import org.opencv.core.Mat;
//import org.opencv.core.Rect;
//import org.opencv.core.Scalar;
//import org.opencv.imgproc.Imgproc;
//import org.openftc.easyopencv.OpenCvPipeline;
//
//@Config
//public
//class samplePickupPipeline extends OpenCvPipeline implements VisionProcessor
//{
//    public static int YminHue = 0, YminSaturation = 60, YminValue = 200 , YmaxHue = 256, YmaxSaturation = 260, YmaxValue = 256  ;
//    public static int RminHue = 0, RminSaturation = 60, RminValue = 200 , RmaxHue = 256, RmaxSaturation = 260, RmaxValue = 256  ;
//    public static int BminHue = 0, BminSaturation = 60, BminValue = 200 , BmaxHue = 256, BmaxSaturation = 260, BmaxValue = 256  ;
//    public static int minHue = 0, minSaturation = 0, minValue = 0 , maxHue = 0, maxSaturation = 0, maxValue = 0  ;
//
//
//
//    int frameWidth, frameHeight;
//
//    int lastResults = 0;
//    Rect firstThird, secondThird, thirdThird;
//    double avgFirstThird, avgSecondThird, avgThirdThird;
//    Mat workingMat1 = new Mat(), workingMat2 = new Mat(), workingMat3 = new Mat();
//    Mat mask1 = new Mat(), mask2 = new Mat(), mask3 = new Mat();
//
//
//
//
//    public samplePickupPipeline(String color) {
//        switch (color) {
//            case "yellow":
//                minHue = YminHue; minSaturation = YminSaturation; minValue = YminValue;
//                maxHue = YmaxHue; maxSaturation = YmaxSaturation; maxValue = YmaxValue;
//                break;
//            case "red":
//                minHue = RminHue; minSaturation = RminSaturation; minValue = RminValue;
//                maxHue = RmaxHue; maxSaturation = RmaxSaturation; maxValue = RmaxValue;
//                break;
//            case "blue":
//                minHue = BminHue; minSaturation = BminSaturation; minValue = BminValue;
//                maxHue = BmaxHue; maxSaturation = BmaxSaturation; maxValue = BmaxValue;
//                break;
//        }
//    }
//
//
//    public int location() {
//        return lastResults;
//    }
//
//    @Override
//    public void init(int width, int height, CameraCalibration calibration) {
//    }
//
//    @Override
//    public Mat processFrame(Mat frame) {
//// Calculate the width and height of the frame
//        frameWidth = frame.width();
//        frameHeight = frame.height();
//
//        // Define the region of interest (ROI) for each third of the screen
//        firstThird = new Rect(0, 0, frameWidth / 3, frameHeight);
//        secondThird = new Rect(frameWidth / 3, 0, frameWidth / 3, frameHeight);
//        thirdThird = new Rect(2 * (frameWidth / 3), 0, frameWidth / 3, frameHeight);
//
//        workingMat1 = frame.clone();
//        // Apply a color mask to the workingMat
//        Imgproc.cvtColor(workingMat1, workingMat2, Imgproc.COLOR_RGB2HSV);
//        Core.inRange(workingMat2, new Scalar(minHue, minSaturation, minValue), new Scalar(maxHue, maxSaturation, maxValue), workingMat3);
//
//        mask1 = workingMat3.submat(firstThird);
//        mask2 = workingMat3.submat(secondThird);
//        mask3 = workingMat3.submat(thirdThird);
//
//        avgFirstThird = Core.countNonZero(mask1);
//        avgSecondThird = Core.countNonZero(mask2);
//        avgThirdThird = Core.countNonZero(mask3);
//
//        if (avgFirstThird > avgSecondThird && avgFirstThird > avgThirdThird) {
//            lastResults = 1;
//        } else if (avgSecondThird > avgThirdThird) {
//            lastResults = 2;
//        } else {
//            lastResults = 3;
//        }
//
//        workingMat1.release();
//        workingMat2.release();
//        workingMat3.release();
//
//        mask1.release();
//        mask2.release();
//        mask3.release();
//
//
//        return frame;
////            return workingMat3.adjustROI(0,0,frameWidth,frameHeight);
////        }
//    }
//
//
//
//    @Override
//    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
//
//    }}