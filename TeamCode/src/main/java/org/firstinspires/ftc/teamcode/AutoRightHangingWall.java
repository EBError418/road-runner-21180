/*
 * copyright 2023 FTC21180
 * */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name=" A - 4 specimens", group="Concept")
public class AutoRightHangingWall extends AutoRightHanging2 {
    @Override
    public void setFloor() {
        floorOrWall = false;
    }
}
