package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="03 Red Front Right + 1 white", group="Concept")
//@Disabled
public class AutoRedFrontRight_twoPlusOne extends AutoRedFrontLeft_fast {

    @Override
    public void setRobotLocation() {
        startLoc = 1;
        leftOrRight = 1;
        pickup2ndWhite = false;
    }
} 
