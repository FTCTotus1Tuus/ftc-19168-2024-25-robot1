package org.firstinspires.ftc.teamcode.team;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class RedFront extends DarienOpModeAuto{
    @Override
    public void runOpMode() throws InterruptedException {
        initCamera(false);
        initControls(true);

        int propPosition;
        //TODO fix arm out for all movements
        //TODO for left and right positions fix centering with offsets
        //TODO move 2 inches more for first centering bc no wiggle
        //TODO find out whats wrong with april tag detection and alignment
        //TODO increase movemnt forward by 1-2 inches depending on above

        waitForStart();

        propPosition = getPropPosition();
        telemetry.addData("Prop", propPosition);
        telemetry.update();
        autoRunMacro("ReadyToPickup");
        setClawPosition("leftClosed"); // makes sure that the purple pixel is picked up
        MoveY(24, 0.3); //centers on spike tile
            setArmPosition(1000, 0.5); // extends the arm

//            sleep(500);
        while(arm.isBusy()){print ("arm pos", arm.getCurrentPosition());}
            setWristPosition("dropGround"); // extends the wrist
        waitForMotors();
        switch (propPosition) {
            case 1:
                AutoRotate(90, 0.3,-1); // turns to spike mark
                autoRunMacro("dropPixel"); // places the purple pixel on the ground
                MoveX(25, 0.3);  // moves 1 tile right to be facing the backdrop
                autoRunMacro("ReadyToPickup"); // returns the wrist
                waitForMotors();
                AutoRotate(-90, 0.3, 1);
                break;
            case 2:
                autoRunMacro("dropPixel"); // places the pixel
                MoveX(-20, 0.3); // strafe left: goes 1 tile towards the pixel piles
                    autoRunMacro("ReadyToPickup"); // returns the wrist
                waitForMotors();
                AutoRotate(0, 0.1 ,1);
                MoveY(25, 0.3);
                waitForMotors();
                AutoRotate(-90, 0.3, 1); // turns towards backdrop
                MoveY(16, 0.3); // moves in line with top case
                waitForMotors();
                break;
            case 3:
                AutoRotate(-90, 0.3,1); // turns to spike mark
                MoveY(3, 0.1);
                waitForMotors();
                autoRunMacro("dropPixel"); // places the pixel
                MoveY(-4, 0.1);
                autoRunMacro("ReadyToPickup"); // returns the wrist
                waitForMotors();
                MoveX(-24, 0.3); // strafe left to the center of the tile, facing the backdrop
                waitForMotors();
                break;
        }
        AutoRotate(-90, 0.3, 0);
        // AT THIS POINT, THE ROBOT SHOULD BE IN THE CENTER OF THE TILE.
        autoRunMacro("ReadyToPickup");
//        setArmPosition(-10,0.1);
        print("second half","");
        MoveY(72,0.5); // moves past stage door towards backdrop
        waitForMotors();
            setClawPosition("closed"); // grabs yellow pixel
        setArmPosition(200, 0.1); // extends the arm a tiny bit
        while (arm.isBusy()) {}
        autoRunMacro("ReadyToDrop"); // extends the wrist
        print("pls no crash","");
        backDropPlace(false, propPosition);
        MoveY(-5, 0.3);
        setWristPosition("dropGround");
        switch (propPosition) {
            case 1:
                MoveX(-18, 0.3);
                waitForMotors();
                break;
            case 2:
                MoveX(-24, 0.3);
                waitForMotors();
                break;
            case 3:
                MoveX(-30, 0.3);
                waitForMotors();
                break;
        }
        MoveY(10, 0.05);
        waitForMotors();
    }
}
