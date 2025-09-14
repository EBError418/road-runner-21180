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
        limelight.setPollRateHz(1); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        waitForStart();   // <-- REQUIRED
        limelight.pipelineSwitch(0);
        int counter = 0;
        while (opModeIsActive()) {
            double[] pythonOutputs = detectAprilTags(counter);
            if (pythonOutputs != null && pythonOutputs.length > 0) {
                if (pythonOutputs[0] == 1) {
                    double x = pythonOutputs[1];
                    double y = pythonOutputs[2];
                    double w = pythonOutputs[3];
                    double h = pythonOutputs[4];
                    int colorCode = (int) pythonOutputs[5];
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
            }
            telemetry.update();
            counter++;
        }
    }

    // Detect any AprilTags in view and send their IDs to telemetry
    public double[] detectAprilTags(int counter) {
        LLResult result = limelight.getLatestResult();

        telemetry.addData("DBG", "counter=%d", counter);

        if (result == null) {
            telemetry.addData("LLResult", "null (no connection or start() not called)");
            telemetry.update();
            return new double[0];
        }

        // Basic validity
        boolean valid = result.isValid();
        telemetry.addData("Result.isValid()", valid);

        // Try to read python outputs
        try {
            double[] py = result.getPythonOutput();
            telemetry.addData("pythonOutputs", py == null ? "null" : ("len=" + py.length));
            telemetry.addData("pythonRaw", py == null ? "null" : Arrays.toString(py));

            boolean allZero = true;
            if (py != null) {
                for (double v : py) { if (v != 0.0) { allZero = false; break; } }
            }
            telemetry.addData("pythonAllZero", py == null ? "null" : Boolean.toString(allZero));
        } catch (Exception e) {
            telemetry.addData("pythonOutputsException", e.toString());
        }

        // Try to read fiducials
        try {
            List<LLResultTypes.FiducialResult> fr = result.getFiducialResults();
            telemetry.addData("fiducials", fr == null ? "null" : fr.size());
        } catch (Exception e) {
            telemetry.addData("fiducialsException", e.toString());
        }

        // Try to read color results
        try {
            List<LLResultTypes.ColorResult> cr = result.getColorResults();
            telemetry.addData("colorResults", cr == null ? "null" : cr.size());
        } catch (Exception e) {
            telemetry.addData("colorResultsException", e.toString());
        }

        telemetry.update();

        // Return pythonOutputs if it's non-null so your main loop can still inspect it
        try {
            double[] py2 = result.getPythonOutput();
            return py2 == null ? new double[0] : py2;
        } catch (Exception e) {
            return new double[0];
        }
    }

}


