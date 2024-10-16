/*
 * copyright 2023 FTC21180
 * */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="Right side auto", group="Concept")
//@Disabled
public class AutoRight extends AutoLeft {
    @Override
    public void setRobotLocation() {
        startLoc = 2;
        leftOrRight = 1;
    }
}
