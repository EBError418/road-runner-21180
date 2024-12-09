/*
 * copyright 2023 FTC21180
 * */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="2 specimens from wall", group="Concept")
public class AutoRightHangingWall extends AutoRightHanging2 {
    @Override
    public void setFloor() {
        floorOrWall = false;
    }
}
