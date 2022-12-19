package org.firstinspires.ftc.teamcode.cameraCalibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
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
import org.openftc.easyopencv.OpenCvInternalCamera2;
import org.openftc.easyopencv.OpenCvPipeline;

@Disabled
@Autonomous
public class phoneCamTuning extends LinearOpMode {

    OpenCvCamera phoneCam;

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

    private final FtcDashboard dash = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId);

        //Allows the camera to turn on
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                phoneCam.setPipeline(new UselessGreenBoxDrawingPipeline());
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
        telemetry.setMsTransmissionInterval(20);
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            dash.getTelemetry();
        }
    }

    class UselessGreenBoxDrawingPipeline extends OpenCvPipeline {
        Mat HSV = new Mat();
        Mat outPut = new Mat();
        Mat rect = new Mat();


        Scalar lowerYellow = new Scalar(25.5, 122, 0);
        Scalar upperYellow = new Scalar(51, 241, 255);

        Scalar lowerPurple = new Scalar(123, 101, 72);
        Scalar upperPurple = new Scalar(147, 171, 151);


        @Override
        public Mat processFrame(Mat input) {
            Scalar lowerGreen = new Scalar(lowerH, lowerS, lowerV);
            Scalar upperGreen = new Scalar(upperH, upperS, upperV);

            Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);

            HSV.copyTo(outPut);

            Rect rectangle = new Rect(rectX, rectY, rectWidth, rectHeight);

            Scalar rectColor = new Scalar(0, 0, 255);

            Core.inRange(HSV, lowerGreen, upperGreen, HSV);

            return HSV;
        }
    }
}
