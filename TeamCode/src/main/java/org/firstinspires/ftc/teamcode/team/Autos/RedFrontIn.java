package org.firstinspires.ftc.teamcode.team.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.team.DarienOpModeAuto;

@Autonomous
public class RedFrontIn extends DarienOpModeAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        initCamera(false);
        initControls(true);

        int propPosition;

        waitForStart();

        propPosition = getPropPosition();
        telemetry.addData("Prop", propPosition);
        telemetry.update();
        autoRunMacro("ReadyToPickup");
        setClawPosition("leftClosed"); // makes sure that the purple pixel is picked up
        MoveY(29, 0.3); //centers on spike tile
            setArmPosition(300, 0.5); // extends the arm

//            sleep(500);
        while(arm.isBusy()){print ("arm pos", arm.getCurrentPosition());}
            setWristPosition("dropGround"); // extends the wrist
        waitForMotors();
        switch (propPosition) {
            case 1:
                AutoRotate(90, 0.3,-1); // turns to spike mark
                MoveY(2.5, 0.2);
                waitForMotors();
                autoRunMacro("dropPixel"); // places the purple pixel on the ground
                waitForMotors();
                MoveX(21.5, 0.3);  // moves 1 tile right to be facing the backdrop
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
                MoveY(22, 0.3);
                waitForMotors();
                AutoRotate(-90, 0.3, 1); // turns towards backdrop
                MoveY(14.5, 0.3); // moves in line with top case
                waitForMotors();
                AutoRotate(-90, 0.3, 0);

                break;
            case 3:
                AutoRotate(-90, 0.3,1); // turns to spike mark
                MoveY(2, 0.1);
                waitForMotors();
                autoRunMacro("dropPixel"); // places the pixel
                MoveY(-4, 0.1);
                autoRunMacro("ReadyToPickup"); // returns the wrist
                waitForMotors();
                MoveX(-18.5, 0.3); // strafe left to the center of the tile, facing the backdrop
                waitForMotors();
                break;
        }

        // AT THIS POINT, THE ROBOT SHOULD BE IN THE CENTER OF THE TILE.
        autoRunMacro("ReadyToPickup");
        setArmPosition(-10,0.1);
        print("second half","");
        MoveY(72,0.5); // moves past stage door towards backdrop
        waitForMotors();
            setClawPosition("closed"); // grabs yellow pixel
        sleep(250);
        setArmPosition(600, 0.3); // extends the arm a tiny bit
        while (arm.isBusy()) {}
        autoRunMacro("dropGround"); // extends the wrist
        print("pls no crash","");

        AutoRotate(-90, 0.3, 0);


        backDropPlace(false, propPosition);
        setWristPosition("dropGround");

        park(false, true, propPosition);

    }
}
