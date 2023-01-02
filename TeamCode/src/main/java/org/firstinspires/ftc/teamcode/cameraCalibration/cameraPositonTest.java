package org.firstinspires.ftc.teamcode.cameraCalibration;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
@Autonomous
public class cameraPositonTest extends LinearOpMode {
    OpenCvCamera phoneCam;

    private static colors color = colors.None;


    public static int rectX = 0;
    public static int rectY = 0;
    public static int rectHeight = 0;
    public static int rectWidth = 0;

    public static double lowerGreenH = 50;
    public static double lowerGreenS = 50;
    public static double lowerGreenV = 25;

    public static double lowerYellowH = 20;
    public static double lowerYellowS = 180;
    public static double lowerYellowV = 50;

    public static double lowerPurpleH = 135;
    public static double lowerPurpleS = 50;
    public static double lowerPurpleV = 25;

    public static double upperGreenH = 90;
    public static double upperGreenS = 255;
    public static double upperGreenV = 255;

    public static double upperYellowH = 30;
    public static double upperYellowS = 255;
    public static double upperYellowV = 255;

    public static double upperPurpleH = 170;
    public static double upperPurpleS = 255;
    public static double upperPurpleV = 255;


    ColorDetectionPipelineV2 pipeline = new ColorDetectionPipelineV2();


    private final FtcDashboard dash = FtcDashboard.getInstance();


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        @SuppressLint("DiscouragedApi")
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId);

        //Allows the camera to turn on
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                phoneCam.setPipeline(pipeline);
            }

            @Override
            public void onError(int errorCode) {
                phoneCam.closeCameraDevice();
                telemetry.addLine("AA2");
                telemetry.addData("error", errorCode);
                telemetry.update();
            }
        });

        FtcDashboard.getInstance().startCameraStream(phoneCam, 30);
        telemetry.setMsTransmissionInterval(100);
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            if (isStopRequested()) {
                return;
            }
        }
    }

    public class ColorDetectionPipelineV2 extends OpenCvPipeline {
        Mat hsv = new Mat();
        Mat mask = new Mat();

        Scalar greenLower = new Scalar(lowerGreenH, lowerGreenS, lowerGreenV);
        Scalar greenUpper = new Scalar(upperGreenH, upperGreenS, upperGreenV);

        Scalar yellowLower = new Scalar(lowerYellowH, lowerYellowS, lowerYellowV);
        Scalar yellowUpper = new Scalar(upperYellowH, upperYellowS, upperYellowV);

        Scalar purpleLower = new Scalar(lowerPurpleH, lowerPurpleS, lowerPurpleV);
        Scalar purpleUpper = new Scalar(upperPurpleH, upperPurpleS, upperPurpleV);

        @Override
        public Mat processFrame(Mat input) {
            // Convert the input image to the HSV color space
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            greenLower = new Scalar(lowerGreenH, lowerGreenS, lowerGreenV);
            greenUpper = new Scalar(upperGreenH, upperGreenS, upperGreenV);

            yellowLower = new Scalar(lowerYellowH, lowerYellowS, lowerYellowV);
            yellowUpper = new Scalar(upperYellowH, upperYellowS, upperYellowV);

            purpleLower = new Scalar(lowerPurpleH, lowerPurpleS, lowerPurpleV);
            purpleUpper = new Scalar(upperPurpleH, upperPurpleS, upperPurpleV);

            // Create a mask for each of the colors we want to detect
            Core.inRange(hsv, greenLower, greenUpper, mask);
            Mat greenMask = mask.clone();

            Core.inRange(hsv, yellowLower, yellowUpper, mask);
            Mat yellowMask = mask.clone();

            Core.inRange(hsv, purpleLower, purpleUpper, mask);
            Mat purpleMask = mask.clone();

            // Set all pixels in the mask to white and all other pixels to black
            Core.bitwise_or(greenMask, yellowMask, mask);
            Core.bitwise_or(purpleMask, mask, mask);
            Core.bitwise_not(mask, mask);
            input.setTo(new Scalar(0, 0, 0), mask);

            // Count the number of non-zero pixels in each of the masks
            int greenCount = Core.countNonZero(greenMask);
            int yellowCount = Core.countNonZero(yellowMask);
            int purpleCount = Core.countNonZero(purpleMask);


            if (greenCount > yellowCount && greenCount > purpleCount) {
                color = colors.green;
                telemetry.addData("Color", color);
                telemetry.update();
            }
            else if (yellowCount > greenCount && yellowCount > purpleCount) {
                color = colors.yellow;
                telemetry.addData("Color", color);
                telemetry.update();
            }
            else if (purpleCount > greenCount && purpleCount > yellowCount) {
                color = colors.purple;
                telemetry.addData("Color", color);
                telemetry.update();
            }
            else {
                color = colors.None;
                telemetry.addData("Color", color);
                telemetry.update();
            }

            // Return the input image
            mask.release();
            hsv.release();
            yellowMask.release();
            purpleMask.release();
            greenMask.release();
            return input;
        }
    }

    public enum colors {
        green,
        purple,
        yellow,
        None
    }
}


