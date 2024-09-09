/*
 * copyright 2023 FTC21180
 * */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Right side parking", group="Concept")
//@Disabled
public class AutoRight_emergency extends AutoLeft_emergency {
    @Override
    public void setRobotLocation() {
        startLoc = 2;
    }
}
