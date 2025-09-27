package org.firstinspires.ftc.teamcode;

import java.io.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="AutoTest", group="Concept")
public class AutoTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        waitForStart();   // <-- REQUIRED
        if (opModeIsActive()) {
            RobotMotion robot = new RobotMotion(hardwareMap);

            robot.moveRobotForward(0.5);
            sleep(2000); // 2 seconds
            robot.stopRobot();
        }
    }
}
