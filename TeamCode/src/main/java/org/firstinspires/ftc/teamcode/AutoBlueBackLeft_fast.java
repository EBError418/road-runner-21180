/*
 * copyright 2023 FTC21180
 * */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="Blue Back Left - 10 sec", group="Concept")
@Disabled
public class AutoBlueBackLeft_fast extends AutoRedBackRight_fast {
    @Override
    public void setRobotLocation() {
        startLoc = 4;
        leftOrRight = -1;
    }
}
