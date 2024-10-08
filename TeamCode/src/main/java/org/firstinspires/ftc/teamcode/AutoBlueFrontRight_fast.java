/*
* copyright 2023 FTC21180
* */
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="Blue Front Right + 2 white", group="Concept")
@Disabled
public class AutoBlueFrontRight_fast extends AutoRedFrontLeft_fast {
    @Override
    public void setRobotLocation() {
        startLoc = 3;
        leftOrRight = 1;
    }
}
