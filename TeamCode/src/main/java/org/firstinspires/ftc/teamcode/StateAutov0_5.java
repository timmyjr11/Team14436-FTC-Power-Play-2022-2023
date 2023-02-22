package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

@Config
@Autonomous
public class StateAutov0_5 extends LinearOpMode {

    private final FtcDashboard dash = FtcDashboard.getInstance();

    static ConfigPos.colors color = ConfigPos.colors.None;
    static ConfigPos.colors previousColor = ConfigPos.colors.None;


    SampleMecanumDrive d;

    OpenCvWebcam blueCam;
    OpenCvWebcam blackCam;

    int liftLevel = 170;

    int coneCounter;

    ConfigPos.side side = ConfigPos.side.tbd;

    ArrayList<Boolean> booleanArray = new ArrayList<>();
    int booleanIncrementer = 0;
    boolean D1UpPressed;
    boolean D1DownPressed;

    boolean ready = false;


    public static int rectX = 127;
    public static int rectY = 45; //95 If right
    public static int rectHeight = 100;
    public static int rectWidth = 75;

    public static double lowerGreenH = 50;
    public static double lowerGreenS = 50;
    public static double lowerGreenV = 25;

    public static double lowerYellowH = 20;
    public static double lowerYellowS = 75;
    public static double lowerYellowV = 50;

    public static double lowerPurpleH = 120;
    public static double lowerPurpleS = 50;
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


    @SuppressWarnings("deprecation")
    @Override
    public void runOpMode() throws InterruptedException {

        d = new SampleMecanumDrive(hardwareMap);


        d.blueLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        d.blackLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        d.blueLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        d.blackLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        d.blueLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        d.blackLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        d.blueServo.setPosition(0);
        d.blackServo.setPosition(0);

        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id",
                        hardwareMap.appContext.getPackageName());


        int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
                .splitLayoutForMultipleViewports(cameraMonitorViewId,
                        2,
                        OpenCvCameraFactory.ViewportSplitMethod.HORIZONTALLY);

        telemetry.addLine("Opening Cameras...");
        telemetry.update();

        blueCam = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), viewportContainerIds[0]);
        blackCam = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), viewportContainerIds[1]);

        blueCam.openCameraDevice();
        blackCam.openCameraDevice();

        telemetry.addLine("Cameras opened");
        telemetry.addLine("Press left on D-pad for left side, press right on D-pad for right side");
        telemetry.update();

        while (true) {
            if (gamepad1.dpad_left) {
                d.setPoseEstimate(StatePoseStorage.leftAuto);
                side = ConfigPos.side.left;
                rectY = 45;
                blackCam.setPipeline(new ColorDetectionPipelineV2());
                blackCam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
                dash.startCameraStream(blackCam, 30);
                break;
            } else if (gamepad1.dpad_right) {
                d.setPoseEstimate(StatePoseStorage.rightAuto);
                rectY = 95;
                side = ConfigPos.side.right;
                blueCam.setPipeline(new ColorDetectionPipelineV2());
                blueCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                dash.startCameraStream(blueCam, 30);
                break;
            }

            if (isStopRequested()) return;
        }



        while(!gamepad1.a) {
            telemetry.addData("Number of cones: ", coneCounter);
            telemetry.addLine("Press up on D-pad to add a cone, press down on D-pad to remove a cone");
            telemetry.addLine("Press A to move on to the next step");
            if (D1UpPressed) {
                coneCounter++;
            }
            if (D1DownPressed) {
                coneCounter--;
            }
            D1UpPressed = ifPressed(gamepad1.dpad_up);
            D1DownPressed = ifPressed(gamepad1.dpad_down);
            booleanIncrementer = 0;
            telemetry.update();
        }

        telemetry.addLine("Creating OpMode, please wait...");
        telemetry.addLine("Somebody once told me the world was gonna roll me, that I ain't the sharpest tool in the shed. - Abraham Lincoln");
        telemetry.update();


        TrajectorySequence leftSide = d.trajectorySequenceBuilder(d.getPoseEstimate())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    d.blueServo.setPosition(1);
                    d.blackServo.setPosition(1);
                })
                .splineToLinearHeading(new Pose2d(-35, -50, Math.toRadians(90)), Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(-0.25, () -> {
                    d.blackLift.setTargetPosition(10000);
                    d.blueLift.setTargetPosition(10000);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                })
                .splineToLinearHeading(new Pose2d(-35, -20, Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-28, -4, Math.toRadians(45)), Math.toRadians(90))
                .waitSeconds(100)
                .build(); // 9 x 8 y


        TrajectorySequence leftSideCones = d.trajectorySequenceBuilder(leftSide.end())
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-47, -15, Math.toRadians(90)), Math.toRadians(180))
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-56.5, -11.75, Math.toRadians(180)), Math.toRadians(180))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-47, -15, Math.toRadians(90)), Math.toRadians(270))
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-35, -11, Math.toRadians(45)), Math.toRadians(45))
                .build();

        ready = true;

        while (!isStarted()) {
            if (color != previousColor) {
                switch (color) {
                    case green:
                        previousColor = ConfigPos.colors.green;
                        break;
                    case purple:
                        previousColor = ConfigPos.colors.purple;
                        break;
                    case yellow:
                        previousColor = ConfigPos.colors.yellow;
                        break;
                }
            }
            if (isStopRequested()) return;
            if (isStarted()) break;
        }

        d.followTrajectorySequence(leftSide);
        for (int i = 0; i < coneCounter; i++) {
            //d.followTrajectorySequence(leftSideCones);
            //liftLevel -= 50;
        }
        // Make a variable that changes the lift height for each loop
    }

    private boolean ifPressed(boolean button) {
        boolean output = false;
        if (booleanArray.size() == booleanIncrementer) {
            booleanArray.add(false);
        }

        boolean buttonWas = booleanArray.get(booleanIncrementer);

        //noinspection PointlessBooleanExpression
        if (button != buttonWas && button == true) {
            output = true;
        }
        booleanArray.set(booleanIncrementer, button);

        booleanIncrementer = booleanIncrementer + 1;
        return output;
    }

    @Config
    public class ColorDetectionPipelineV2 extends OpenCvPipeline {
        //Mat objects to hold the original image,
        // the filtered image, and the region of interest
        private final Mat hsvMat = new Mat();
        private final Mat mask = new Mat();
        private final Mat filtered = new Mat();
        private final Mat filteredG = new Mat();

        // Declare the rectangle to be used for processing
        Rect rect  = new Rect(rectX, rectY, rectWidth, rectHeight);

        // Declare the limits and colors needed for processing

        private final Scalar black = new Scalar(0, 0, 0);



        @Override
        public Mat processFrame(Mat input) {

            Scalar lowerGreen = new Scalar(lowerGreenH, lowerGreenS, lowerGreenV);
            Scalar upperGreen = new Scalar(upperGreenH, upperGreenS, upperGreenV);

            Scalar lowerYellow = new Scalar(lowerYellowH, lowerYellowS, lowerYellowV);
            Scalar upperYellow = new Scalar(upperYellowH, upperYellowS, upperYellowV);

            Scalar lowerPurple = new Scalar(lowerPurpleH, lowerPurpleS, lowerPurpleV);
            Scalar upperPurple = new Scalar(upperPurpleH, upperPurpleS, upperPurpleV);

            rect = new Rect(new Point(rectX, rectY), new Size(rectWidth, rectHeight));

            // Convert the color to HSV and create a submat for
            // Region of interest
            Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);
            Mat roi = new Mat(hsvMat, rect);

            // Filter for yellow
            Core.inRange(roi, lowerYellow, upperYellow, mask);
            int yellowCount = Core.countNonZero(mask);

            // Filter for green
            Core.inRange(roi, lowerGreen, upperGreen, filteredG);
            int greenCount = Core.countNonZero(filteredG);

            // Filter for purple
            Core.inRange(roi, lowerPurple, upperPurple, filtered);
            int purpleCount = Core.countNonZero(filtered);

            // Combine the filters together and set extra colors to black
            Core.bitwise_or(mask, filteredG, mask);
            Core.bitwise_or(mask, filtered, mask);
            Core.bitwise_not(mask, mask);
            roi.setTo(black, mask);

            // Convert back to RGB and place the rectangle
            Imgproc.cvtColor(hsvMat, input, Imgproc.COLOR_HSV2RGB);
            Imgproc.rectangle(input, rect.tl(), rect.br(), new Scalar(0, 255, 0), 2);

            // Count the number of non-zero pixels in each of the masks
            if (greenCount > yellowCount && greenCount > purpleCount) {
                color = ConfigPos.colors.green;
                telemetry.addData("Color", color);
            }
            else if (yellowCount > greenCount && yellowCount > purpleCount) {
                color = ConfigPos.colors.yellow;
                telemetry.addData("Color", color);
            }
            else if (purpleCount > greenCount && purpleCount > yellowCount) {
                color = ConfigPos.colors.purple;
                telemetry.addData("Color", color);
            }
            else {
                color = ConfigPos.colors.None;
                telemetry.addData("Color", color);
            }

            telemetry.addData("Side", side);
            telemetry.addData("Cone count", coneCounter);
            telemetry.addData("Robot ready? ", ready);
            telemetry.update();
            return input;
        }
    }
}