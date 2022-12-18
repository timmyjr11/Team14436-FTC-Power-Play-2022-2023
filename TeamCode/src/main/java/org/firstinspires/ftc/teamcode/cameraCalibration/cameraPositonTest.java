package org.firstinspires.ftc.teamcode.cameraCalibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Disabled
@Autonomous
public class cameraPositonTest extends LinearOpMode {
    OpenCvCamera blueCam;
    OpenCvCamera blackCam;

    private final FtcDashboard dash = FtcDashboard.getInstance();

    public static int rectX = 0;
    public static int rectY = 0;
    public  static int rectHeight = 0;
    public static int rectWidth = 0;

    public static double lowerH = 0;
    public static double lowerS = 0;
    public static  double lowerV = 0;

    public static double upperH = 180;
    public static double upperS = 255;
    public static double upperV = 255;




    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
                .splitLayoutForMultipleViewports(cameraMonitorViewId, 2, OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY);

        blueCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), viewportContainerIds[0]);
        blackCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), viewportContainerIds[1]);

        blueCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                blueCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                blueCam.setPipeline(new UselessGreenBoxDrawingPipeline());
            }

            @Override
            public void onError(int errorCode) {
                blueCam.closeCameraDevice();
                telemetry.addLine("AA1");
                telemetry.addData("error", errorCode);

                telemetry.update();
            }
        });

        blackCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                blackCam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {
                blackCam.closeCameraDevice();
                telemetry.addLine("AA2");
                telemetry.addData("error", errorCode);
                telemetry.update();
            }
        });


        dash.startCameraStream(blueCam, 15);
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad1.a) {
                telemetry.clearAll();
                dash.stopCameraStream();
                dash.startCameraStream(blueCam, 15);
                telemetry.addLine("BlueCam");
            } else if (gamepad1.b) {
                telemetry.clearAll();
                dash.stopCameraStream();
                dash.startCameraStream(blackCam, 15);
                telemetry.addLine("BlackCam");
            }
            telemetry.update();
        }
    }

    class UselessGreenBoxDrawingPipeline extends OpenCvPipeline {
        Mat HSV = new Mat();
        Mat outPut = new Mat();
        Mat rect = new Mat();

        Scalar lowerGreen = new Scalar(lowerH, lowerS, lowerV);
        Scalar upperGreen = new Scalar(upperH, upperS, upperV);

        Scalar lowerYellow = new Scalar(25.5, 122, 0);
        Scalar upperYellow = new Scalar(51, 241, 255);

        Scalar lowerPurple = new Scalar(123, 101, 72);
        Scalar upperPurple = new Scalar(147, 171, 151);


        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);

            HSV.copyTo(outPut);

            Rect rectangle = new Rect(rectX, rectY, rectWidth, rectHeight);

            Scalar rectColor = new Scalar(0, 0, 255);

            Core.inRange(input, lowerGreen, upperGreen, HSV);

            return HSV;

        }
    }
}
