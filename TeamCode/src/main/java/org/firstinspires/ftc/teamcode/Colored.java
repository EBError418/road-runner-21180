package org.firstinspires.ftc.teamcode;
import java.util.ArrayList;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Colored", group="Concept")
public class Colored extends LinearOpMode {
    Limelight3A limelight;
    String pattern = "none";

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        waitForStart();   // <-- REQUIRED
        limelight.pipelineSwitch(2);
        while (opModeIsActive()) {
            // receive chunked results
            ArrayList<ArrayList<Double>> chunkedResults = chunkResult();
            if (chunkedResults.size() > 0) {
                for (ArrayList<Double> result : chunkedResults) {
                    if (result.size() == 5) {
                        double colorCode = result.get(0);
                        double x = result.get(1);
                        double y = result.get(2);
                        double w = result.get(3);
                        double h = result.get(4);
//                        if (colorCode == 1) {
//                            telemetry.addData("Green Artifact", "x: %.1f, y: %.1f, w: %.1f, h: %.1f, area: %.1f", x, y, w, h, w*h);
//                        } else if (colorCode == 2) {
//                            telemetry.addData("Purple Artifact", "x: %.1f, y: %.1f, w: %.1f, h: %.1f, area: %.1f", x, y, w, h, w*h);
//                        } else
                        if (colorCode == 20){
                            telemetry.addData("Blue Goal", "x: %.1f, y: %.1f, w: %.1f, h: %.1f, area: %.1f", x, y, w, h, w*h);
                        } else if (colorCode == 21){
                            telemetry.addData("GPP Pattern", "x: %.1f, y: %.1f, w: %.1f, h: %.1f, area: %.1f", x, y, w, h, w*h);
                        } else if (colorCode == 22){
                            telemetry.addData("PGP Pattern", "x: %.1f, y: %.1f, w: %.1f, h: %.1f, area: %.1f", x, y, w, h, w*h);
                        } else if (colorCode == 23){
                            telemetry.addData("PPG Pattern", "x: %.1f, y: %.1f, w: %.1f, h: %.1f, area: %.1f", x, y, w, h, w*h);
                        } else if (colorCode == 24){
                            telemetry.addData("Red Goal", "x: %.1f, y: %.1f, w: %.1f, h: %.1f, area: %.1f", x, y, w, h, w*h);
                        }
                    }
                }
            }
            telemetry.update();
        }
    }
    public ArrayList<ArrayList<Double>> chunkResult() {
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


