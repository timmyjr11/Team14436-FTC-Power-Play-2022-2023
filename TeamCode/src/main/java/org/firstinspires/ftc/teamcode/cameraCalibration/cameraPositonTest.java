package org.firstinspires.ftc.teamcode.cameraCalibration;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
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


    public static int rectX = 50;
    public static int rectY = 50;
    public static int rectHeight = 25;
    public static int rectWidth = 25;

    public static double lowerGreenH = 50;
    public static double lowerGreenS = 50;
    public static double lowerGreenV = 25;

    public static double lowerYellowH = 20;
    public static double lowerYellowS = 200;
    public static double lowerYellowV = 50;

    public static double lowerPurpleH = 120;
    public static double lowerPurpleS = 0;
    public static double lowerPurpleV = 0;

    public static double upperGreenH = 90;
    public static double upperGreenS = 255;
    public static double upperGreenV = 255;

    public static double upperYellowH = 35;
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
        //Mat objects to hold the original image,
        // the filtered image, and the region of interest
        private final Mat hsvMat = new Mat();
        private final Mat mask = new Mat();
        private final Mat filtered = new Mat();

        // Declare the rectangle to be used for processing
        Rect rect  = new Rect(rectX, rectY, rectWidth, rectHeight);

        // Declare the limits and colors needed for processing
        Scalar lowerGreen = new Scalar(lowerGreenH, lowerGreenS, lowerGreenV);
        Scalar upperGreen = new Scalar(upperGreenH, upperGreenS, upperGreenV);

        Scalar lowerYellow = new Scalar(lowerYellowH, lowerYellowS, lowerYellowV);
        Scalar upperYellow = new Scalar(upperYellowH, upperYellowS, upperYellowV);

        Scalar lowerPurple = new Scalar(lowerPurpleH, lowerPurpleS, lowerPurpleV);
        Scalar upperPurple = new Scalar(upperPurpleH, upperPurpleS, upperPurpleV);

        private final Scalar black = new Scalar(0, 0, 0);



        @Override
        public Mat processFrame(Mat input) {

            //Create the rectangle
            int x = (input.width() - 320) / 2;
            int y = (input.height() - 240) / 2;
            rect = new Rect(new Point(x + 135, y + 95), new Size(100, 100));

            // Convert the color to HSV and create a submat for
            // Region of interest
            Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);
            Mat roi = new Mat(hsvMat, rect);

            // Filter for yellow
            Core.inRange(roi, lowerYellow, upperYellow, mask);
            int yellowCount = Core.countNonZero(mask);

            // Filter for green
            Core.inRange(roi, lowerGreen, upperGreen, filtered);
            int greenCount = Core.countNonZero(filtered);

            // Filter for purple
            Core.inRange(roi, lowerPurple, upperPurple, filtered);
            int purpleCount = Core.countNonZero(filtered);

            // Combine the filters together and set extra colors to black
            Core.bitwise_or(mask, filtered, mask);
            Core.bitwise_not(mask, mask);
            roi.setTo(black, mask);

            // Convert back to RGB and place the rectangle
            Imgproc.cvtColor(hsvMat, input, Imgproc.COLOR_HSV2RGB);
            Imgproc.rectangle(input, rect.tl(), rect.br(), new Scalar(0, 255, 0), 2);

            // Count the number of non-zero pixels in each of the masks
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


