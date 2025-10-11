package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public final class Colored {
    private final Limelight3A limelight;

    public Colored(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);  // poll 100x per second
        limelight.start();
        limelight.pipelineSwitch(2);
    }
    public double[] returnPosition() {
        // Get latest result from Limelight
        LLResult result = limelight.getLatestResult();
        if (result == null) return null;

        double[] pythonOutputs = result.getPythonOutput();
        if (pythonOutputs == null || pythonOutputs.length < 5) return null;

        // Each detection is 5 values long: [goal code, x, y, w, h]
        for (int i = 0; i < pythonOutputs.length; i += 5) {
            double goalCode = pythonOutputs[i];
            double x = pythonOutputs[i + 1];
            double y = pythonOutputs[i + 2];
            double w = pythonOutputs[i + 3];
            double h = pythonOutputs[i + 4];

            if (goalCode == 20 || goalCode == 24) {
                // Return array of all 5 values
                return new double[] {goalCode, x, y, w, h};
            }
        }

        // No valid detection found
        return null;
    }
    public double returnId() {
        LLResult result = limelight.getLatestResult();
        if (result == null) return 0;

        double[] pythonOutputs = result.getPythonOutput();
        if (pythonOutputs == null || pythonOutputs.length < 5) return 0;
        // Each detection is 5 values long: [colorCode, x, y, w, h]
        for (int i = 0; i < pythonOutputs.length; i += 5) {
            double colorCode = pythonOutputs[i];
            // x, y, w, h not needed right now
            if (colorCode == 21 || colorCode == 22 || colorCode == 23) {
                return colorCode;
            }
        }

        return 0; // no valid detection
    }
}
