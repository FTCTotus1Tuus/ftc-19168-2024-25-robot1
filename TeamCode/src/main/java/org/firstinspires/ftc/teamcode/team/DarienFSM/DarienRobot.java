package org.firstinspires.ftc.teamcode.team.DarienFSM;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;


@Config
public abstract class DarienRobot extends CommandOpMode {

    public enum OpModeType {
        TELEOP, AUTO
    }

//    public DarienRobot(OpModeType type) {
//        if (type == OpModeType.TELEOP) {
//            initTele();
//        } else {
//            initAuto();
//        }
//    }
}

