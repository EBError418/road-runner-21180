//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.LLStatus;
//import com.qualcomm.hardware.limelightvision.LLResultTypes;
//
//import java.util.HashMap;
//import java.util.List;
//import java.util.Arrays;
//
//@TeleOp(name="SwitchLimeLightPipelines", group = "Concept")
////@Disabled
//public class LimeLightSwitchPipelines extends LinearOpMode
//{
//    Limelight3A limelight;
//
//    HashMap<Integer, List<String>> tags = new HashMap<>();
//    {
//        tags.put(20, Arrays.asList("Blue"));
//        tags.put(21, Arrays.asList("G", "P", "P"));
//        tags.put(22, Arrays.asList("P", "G", "P"));
//        tags.put(23, Arrays.asList("P", "P", "G"));
//        tags.put(24, Arrays.asList("Red"));
//    }
//
//    @Override
//    public void init() {
//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
//        limelight.start(); // This tells Limelight to start looking!
//    }
//
//    @Override public void runOpMode()
//    {
////        Pipeline 0 is color, pipeline 1 is AprilTags.
//        limelight.pipelineSwitch(1);
//
////        Detect pattern via AprilTags
//        LLResult result = limelight.getLatestResult();
//
//        List fiducialResults = result.getFiducialResults();
//
//        if (!result.getStatus() == LLStatus.LL_OK) {
//            telemetry.addData("Pattern", "No Fiducials Found");
//            return;
//        }
//
//        int id = result.getFiducialID();
//        if (!tags.containsKey(id)) {
//            telemetry.addData("Pattern", "Unknown ID: " + id);
//            return;
//        }
//
//        List<String> pattern = tags.get(id);
//        if (pattern[0] == "Blue" || pattern[0] == "Red") {
//            return;
//        }
//
//        telemetry.addData("Pattern", pattern);
//
////        Switch pipeline to 0
//        limelight.pipelineSwitch(0);
//
////        Find balls, drive to them, repeat for all 3.
//
////        Shoot balls.
//
//    }
//}
