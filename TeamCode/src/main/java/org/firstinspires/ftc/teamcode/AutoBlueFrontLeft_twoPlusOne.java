package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="03 Blue Front Left + 1 white", group="Concept")
//@Disabled
public class AutoBlueFrontLeft_twoPlusOne extends AutoRedFrontLeft_fast {
    @Override
    public void setRobotLocation() {
        startLoc = 3;
        leftOrRight = -1;
        pickup2ndWhite = false;
    }
}