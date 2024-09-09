/*
 * copyright 2023 FTC21180
 * */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="Red Back Left - 10 sec", group="Concept")
@Disabled
public class AutoRedBackLeft extends AutoRedBackRight_fast {
    @Override
    public void setRobotLocation() {
        startLoc = 2;
        leftOrRight = -1;
    }
}
