package org.firstinspires.ftc.teamcode.team;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class BlueFront extends DarienOpModeAuto{
    @Override
    public void runOpMode() throws InterruptedException {
        initCamera(true);
        initControls(true);

        int propPosition;

        waitForStart();

        propPosition = getPropPosition();
        telemetry.addData("Prop", teamPropMaskPipeline.getLastResults());
        telemetry.update();
        autoRunMacro("ReadyToPickup");
        setClawPosition("leftClosed"); // makes sure that the purple pixel is picked up
        MoveY(24, 0.3); //centers on spike tile
            sleep(350);
            setArmPosition(250, 0.3); // extends the arm
            sleep(500);
            setWristPosition("dropGround"); // extends the wrist
        waitForMotors();
        switch (propPosition) {
            //TODO modify 1 and 3 to move forward and back proper amounts
            case 3:
                AutoRotate(-90, 0.3,1); // turns to spike mark
                MoveY(3, 0.1); // move toward the spike mark
                waitForMotors();
                autoRunMacro("dropPixel"); // places the purple pixel on the ground
                MoveY(-1, 0.1);
                waitForMotors();
                MoveX(-24, 0.3);  // strafe left to center on the tile
                autoRunMacro("ReadyToPickup"); // returns the wrist
                waitForMotors();
                AutoRotate(90, 0.3, -1);
                //MoveY(-0.5, 0.1); // back up a tad
                //waitForMotors();
                break;
            case 2:
                autoRunMacro("dropPixel"); // places the pixel
                MoveX(20, 0.3); // goes 1 tile towards the pixel piles
                    autoRunMacro("ReadyToPickup"); // returns the wrist
                waitForMotors();
                AutoRotate(0, 0.1 ,1);
                MoveY(24, 0.3);
                waitForMotors();
                AutoRotate(90, 0.3, -1); // turns towards backdrop
                MoveY(18, 0.3); // moves in line with top case
                waitForMotors();
                break;
            case 1:
                AutoRotate(90, 0.3,-1); // turns to spike mark
                MoveY(0.5, 0.1);
                waitForMotors();
                autoRunMacro("dropPixel"); // places the pixel
                MoveY(-3.5, 0.1);
                autoRunMacro("ReadyToPickup"); // returns the wrist
                waitForMotors();
                MoveX(24, 0.3); // moves in line with top case
                waitForMotors();
                break;
        }
        // AT THIS POINT, THE ROBOT SHOULD BE IN THE CENTER OF THE TILE IN FRONT OF THE STAGE DOOR.
        autoRunMacro("ReadyToPickup");
        setArmPosition(-10,0.1);
        AutoRotate(90, 0.3, 0);
        MoveY(72, 0.5); // moves past stage door towards backdrop
        waitForMotors();
            setClawPosition("closed"); // grabs yellow pixel
//        AutoRotate(90, 0.3, -1);
        setArmPosition(200, 0.1); // extends the arm a tiny bit
        while (leftArm.isBusy()) {}
        autoRunMacro("ReadyToDrop"); // extends the wrist
        print("pls no crash","");
        backDropPlace(true, propPosition);
        switch (propPosition) {
            //TODO fix distance strafed to and fro for placing
            case 3:
                MoveX(21, 0.3);
                waitForMotors();
                break;
            case 2:
                MoveX(24, 0.3);
                waitForMotors();
                break;
            case 1:
                MoveX(32, 0.3);
                waitForMotors();
                break;
        }
        MoveY(10, 0.05);
        waitForMotors();
    }
}
