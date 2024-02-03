/*
 * copyright 2023 FTC21180
 * */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Red Front Left", group="Concept")
//@Disabled
public class AutoRedFrontLeft extends AutoRedBackLeft {
    @Override
    public void setRobotLocation() {
        startLoc = 1;
        leftOrRight = -1;
    }
}
