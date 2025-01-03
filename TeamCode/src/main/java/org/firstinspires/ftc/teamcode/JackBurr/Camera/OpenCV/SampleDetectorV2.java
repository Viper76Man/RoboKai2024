package org.firstinspires.ftc.teamcode.JackBurr.Camera.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;


@TeleOp
public class SampleDetectorV2 extends OpMode {
    public SampleDetectorToolkit toolkit;
    public SampleDetectorVisionPortalToolkit visionToolkit;
    public ColorBlobLocatorProcessor red;
    public ColorBlobLocatorProcessor neutral;
    public ColorBlobLocatorProcessor blue;
    public List<ColorBlobLocatorProcessor> processorList = new ArrayList<>();
    public List<ColorBlobLocatorProcessor.Blob> redList = new ArrayList<>();
    public List<ColorBlobLocatorProcessor.Blob> neutralList = new ArrayList<>();
    public List<ColorBlobLocatorProcessor.Blob> blueList = new ArrayList<>();
    public List<SampleDetection> masterList = new ArrayList<>();
    public VisionPortal portal;
    public int MIN_AREA = 30000;
    public int MAX_AREA = 500000;
    public Point center;
    public SampleDetection closestSample;
    public ElapsedTime buttonTimer = new ElapsedTime();
    public enum SelectedColors {
        RED_ONLY,
        BLUE_ONLY,
        YELLOW_ONLY,
        RED_AND_BLUE,
        RED_AND_YELLOW,
        BLUE_AND_YELLOW,
        ALL
    }
    public double sampleAngle;
    @Override
    public void init() {
        telemetry.setMsTransmissionInterval(50);
        toolkit = new SampleDetectorToolkit(hardwareMap);
        visionToolkit = new SampleDetectorVisionPortalToolkit(hardwareMap);
        red = toolkit.getNewProcessor(ColorRange.RED,  MIN_AREA, MAX_AREA);
        neutral = toolkit.getNewProcessor(ColorRange.YELLOW,  MIN_AREA, MAX_AREA);
        blue = toolkit.getNewProcessor(ColorRange.BLUE,  MIN_AREA, MAX_AREA);
        processorList.add(red);
        processorList.add(neutral);
        processorList.add(blue);
        portal = visionToolkit.createVisionPortal(hardwareMap, processorList, "Webcam 1");
        center = toolkit.getCenter(1280, 960);
    }

    @Override
    public void init_loop(){
        masterList.clear();
        redList = toolkit.filterByArea(MIN_AREA, MAX_AREA, red.getBlobs());
        neutralList = toolkit.filterByArea(MIN_AREA, MAX_AREA, neutral.getBlobs());
        blueList = toolkit.filterByArea(MIN_AREA, MAX_AREA, blue.getBlobs());
        masterList = toolkit.addToSampleDetectionList(masterList, ColorRange.RED, redList);
        masterList = toolkit.addToSampleDetectionList(masterList, ColorRange.YELLOW, neutralList);
        masterList = toolkit.addToSampleDetectionList(masterList, ColorRange.BLUE, blueList);
        Iterator<SampleDetection> iterator = masterList.iterator();
       /* while (iterator.hasNext()) {
            SampleDetection detection = iterator.next();
            if (!detection.exists) {
                iterator.remove();
                continue;
            }
            telemetry.addLine("Detected " + toolkit.getColorName(detection) + " sample/specimen:");
            telemetry.addLine("\t Width: " + detection.width);
            telemetry.addLine("\t Height: " + detection.height);
            telemetry.addLine("\t Angle: " + detection.angle);
            telemetry.addLine("\t Rotation Needed: " + toolkit.findNeededRotationDegrees(detection.boxFit));
        }
        */
        closestSample = toolkit.findClosestSample(center, masterList);
        sampleAngle = closestSample.angle;
        if (closestSample.width < closestSample.height) {
            sampleAngle = sampleAngle - 90;
            closestSample.sampleRotation = SampleDetection.SampleRotation.HORIZONTAL;
        }
        else {
            closestSample.sampleRotation = SampleDetection.SampleRotation.VERTICAL;
        }
        telemetry.addLine("Detected " + toolkit.getColorName(closestSample) + " sample/specimen:");
        telemetry.addLine("\t X Position: " + closestSample.x);
        telemetry.addLine("\t Y Position: " + closestSample.y);
        telemetry.addLine("\t Width: " + closestSample.width);
        telemetry.addLine("\t Height: " + closestSample.height);
        telemetry.addLine("\t Angle: " + sampleAngle);
        telemetry.addLine("\t Orientation: " + closestSample.sampleRotation.name());
        telemetry.addLine("\t Rotation Needed: " + toolkit.findNeededRotationDegrees(closestSample.boxFit));
    }

    @Override
    public void loop() {
        portal.stopStreaming();
    }
}
