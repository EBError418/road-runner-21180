package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Far Red shoot 2026", group = "Concept")
public class AutoFarRed2026 extends AutoFarBlue2026 {
    @Override
    public void setSide() {
        leftOrRight = -1;
    }
}