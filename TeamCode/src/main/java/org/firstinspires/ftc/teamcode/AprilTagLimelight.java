package org.firstinspires.ftc.teamcode;
import java.io.*;
import java.util.Arrays;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;

import java.util.List;


@Autonomous(name="AprilTagTest", group="Concept")
public class AprilTagLimelight extends LinearOpMode {
    Limelight3A limelight;

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        waitForStart();   // <-- REQUIRED
        limelight.pipelineSwitch(0);
        while (opModeIsActive()) {
            // from old function start
            LLResult result = limelight.getLatestResult();
            double[] pythonOutputs = result.getPythonOutput();
            if (pythonOutputs != null && pythonOutputs.length > 0) {
                double x = pythonOutputs[1];
                double y = pythonOutputs[2];
                double w = pythonOutputs[3];
                double h = pythonOutputs[4];
                int colorCode = (int) pythonOutputs[0];
                if (colorCode == 1) {
                    telemetry.addData("Green Artifact", "x: %.1f, y: %.1f, w: %.1f, h: %.1f", x, y, w, h);
                } else if (colorCode == 2) {
                    telemetry.addData("Purple Artifact", "x: %.1f, y: %.1f, w: %.1f, h: %.1f", x, y, w, h);
                } else {
                    telemetry.addData("Unknown Artifact", "x: %.1f, y: %.1f, w: %.1f, h: %.1f", x, y, w, h);
                }
            } else {
                telemetry.addData("No ball detected", "");
            }
            //end
            telemetry.update();
        }
    }
}


