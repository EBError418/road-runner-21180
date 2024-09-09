/*
* copyright 2023 FTC21180
* */
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="Blue Front Right", group="Concept")
@Disabled
public class AutoBlueFrontRight extends AutoLeft {
    @Override
    public void setRobotLocation() {
        startLoc = 3;
        leftOrRight = 1;
    }
}
