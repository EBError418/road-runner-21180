package org.firstinspires.ftc.teamcode;
import java.util.ArrayList;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public final class Colored{
    Limelight3A limelight;
    String pattern = "none";

    public Colored(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        limelight.pipelineSwitch(2);
        // receive chunked results



    }
    public double returnId() {
        ArrayList<ArrayList<Double>> chunkedResults = chunkResult();
        if (!chunkedResults.isEmpty()) {
            for (ArrayList<Double> result : chunkedResults) {
                if (result.size() == 5) {
                    double colorCode = result.get(0);
                    double x = result.get(1);
                    double y = result.get(2);
                    double w = result.get(3);
                    double h = result.get(4);
//                       if (colorCode == 1) {
//                           telemetry.addData("Green Artifact", "x: %.1f, y: %.1f, w: %.1f, h: %.1f, area: %.1f", x, y, w, h, w*h);
//                       } else if (colorCode == 2) {
//                           telemetry.addData("Purple Artifact", "x: %.1f, y: %.1f, w: %.1f, h: %.1f, area: %.1f", x, y, w, h, w*h);
//                       } else
                    if (colorCode == 21 || colorCode == 22 || colorCode == 23) {
                        return colorCode;
                    }
                }
            }
        }
        return 0;
    }
    private ArrayList<ArrayList<Double>> chunkResult() {
        LLResult result = limelight.getLatestResult();
        double[] pythonOutputs = result.getPythonOutput();
        // pythonOutputs is a long array where each group of 5 values represents one detected object
        ArrayList<ArrayList<Double>> chunkedResults = new ArrayList<>();
        for (int i = 0; i < pythonOutputs.length; i += 5) {
            ArrayList<Double> chunk = new ArrayList<>();
            for (int j = i; j < i + 5 && j < pythonOutputs.length; j++) {
                chunk.add(pythonOutputs[j]);
            }
            chunkedResults.add(chunk);
        }
        return chunkedResults;
    }
}


