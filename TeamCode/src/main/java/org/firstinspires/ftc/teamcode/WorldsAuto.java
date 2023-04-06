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
public class WorldsAuto extends LinearOpMode {
    private final FtcDashboard dash = FtcDashboard.getInstance();

    static WorldsConfig.colors color = WorldsConfig.colors.None;

    SampleMecanumDrive d;

    OpenCvWebcam blueCam;
    OpenCvWebcam blackCam;

    int liftLevel = 400;
    int topJunction = 1000;

    int leftForward = 0;
    int leftSideLevel = 0;
    int rightForward = 0;
    int rightSideLevel = 0;

    int coneCounter;

    WorldsConfig.side side = WorldsConfig.side.tbd;

    ArrayList<Boolean> booleanArray = new ArrayList<>();
    TrajectorySequence[] array;

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

        d.blueArm.setPosition(0);
        d.blackArm.setPosition(0);
        d.rotateServo.setPosition(0);
        d.blueGripper.setPosition(0);
        d.blackGripper.setPosition(0);

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
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), viewportContainerIds[0]);
        blackCam = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), viewportContainerIds[1]);

        blueCam.openCameraDevice();
        blackCam.openCameraDevice();

        telemetry.addLine("Cameras opened");
        telemetry.addLine("Press left on D-pad for left side, press right on D-pad for right side");
        telemetry.update();

        while (true) {
            if (gamepad1.dpad_left) {
                d.setPoseEstimate(WorldsConfig.leftAuto);
                side = WorldsConfig.side.left;
                rectY = 45;
                blackCam.setPipeline(new ColorDetectionPipeline());
                blackCam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
                dash.startCameraStream(blackCam, 30);
                break;
            } else if (gamepad1.dpad_right) {
                d.setPoseEstimate(WorldsConfig.rightAuto);
                rectY = 95;
                side = WorldsConfig.side.right;
                blueCam.setPipeline(new ColorDetectionPipeline());
                blueCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                dash.startCameraStream(blueCam, 30);
                break;
            }
            if (isStopRequested()) return;
        }

        while (!gamepad1.a) {
            telemetry.addData("Number of cones: ", coneCounter);
            telemetry.addLine("Press up on D-pad to add a cone, press down on D-pad to remove a cone");
            telemetry.addLine("Press A to move on to the next step");
            if (D1UpPressed && coneCounter < 5) {
                coneCounter++;
            } else if (D1DownPressed && coneCounter > 0) {
                coneCounter--;
            }
            D1UpPressed = ifPressed(gamepad1.dpad_up);
            D1DownPressed = ifPressed(gamepad1.dpad_down);
            booleanIncrementer = 0;
            telemetry.update();
            if (isStopRequested()) return;
        }

        array = (side == WorldsConfig.side.left) ? buildLeftSide() : buildRightSide();

        ready = true;

        while (!isStarted()) if (isStopRequested()) return;

        if (side == WorldsConfig.side.left) {
            blackCam.stopStreaming();
        } else {
            blueCam.stopStreaming();
        }

        d.followTrajectorySequence(array[0]);
        d.followTrajectorySequence(array[1]);
        switch (color) {
            case green:
                d.followTrajectorySequence(array[2]);
                break;
            case yellow:
                d.followTrajectorySequence(array[3]);
                break;
            case purple:
                d.followTrajectorySequence(array[4]);
                break;
        }
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

    private TrajectorySequence[] buildLeftSide() {

        TrajectorySequence leftSide = d.trajectorySequenceBuilder(d.getPoseEstimate())
                .splineToSplineHeading(new Pose2d(-33, -47, Math.toRadians(180)), Math.toRadians(90))

                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    d.blueGripper.setPosition(0.8);
                    d.blackGripper.setPosition(0.8);
                })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    d.blackLift.setTargetPosition(topJunction);
                    d.blueLift.setTargetPosition(topJunction);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                    d.blueArm.setPosition(1);
                    d.blackArm.setPosition(1);
                    d.rotateServo.setPosition(1);
                })
                .splineToSplineHeading(new Pose2d(-33, -20, Math.toRadians(180)), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-28, -4, Math.toRadians(225)), Math.toRadians(45))
                .waitSeconds(0.2)
                .UNSTABLE_addDisplacementMarkerOffset(-0.1, () -> {
                    d.blackGripper.setPosition(0);
                    d.blueGripper.setPosition(0);
                })
                .build(); // 9 x 8 y


        TrajectorySequence leftSideCones = d.trajectorySequenceBuilder(leftSide.end())
                // Grab first cone from stack
                .splineToSplineHeading(new Pose2d(-47, -8, Math.toRadians(180)), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    d.blueArm.setPosition(0);
                    d.blackArm.setPosition(0);
                    d.rotateServo.setPosition(0);
                    d.blackLift.setTargetPosition(liftLevel);
                    d.blueLift.setTargetPosition(liftLevel);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                })
                .splineToSplineHeading(new Pose2d(-56.5, -8, Math.toRadians(180)), Math.toRadians(180))
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(-0.25, () -> {
                    d.blackGripper.setPosition(0.8);
                    d.blueGripper.setPosition(0.8);
                })
                // Place first cone
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-47, -8, Math.toRadians(180)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    d.blueArm.setPosition(1);
                    d.blackArm.setPosition(1);
                    d.rotateServo.setPosition(1);
                    d.blackLift.setTargetPosition(topJunction);
                    d.blueLift.setTargetPosition(topJunction);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                })
                .splineToSplineHeading(new Pose2d(-28, -4, Math.toRadians(225)), Math.toRadians(45))
                .waitSeconds(0.2)
                .UNSTABLE_addDisplacementMarkerOffset(-0.2, () -> {
                    d.blackGripper.setPosition(0);
                    d.blueGripper.setPosition(0);
                })
                .setReversed(false)
                .build();


        TrajectorySequence leftSideParkLeft = d.trajectorySequenceBuilder(leftSide.end())
                .splineToSplineHeading(new Pose2d(-33, -31, Math.toRadians(270)), Math.toRadians(270))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    d.blueArm.setPosition(0);
                    d.blackArm.setPosition(0);
                    d.rotateServo.setPosition(0);
                    d.blackLift.setTargetPosition(0);
                    d.blueLift.setTargetPosition(0);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                })
                .lineToConstantHeading(new Vector2d(-56, -31))
                .build(); // If left

        TrajectorySequence leftSideParkMiddle = d.trajectorySequenceBuilder(leftSide.end())
                .splineToSplineHeading(new Pose2d(-33, -31, Math.toRadians(270)), Math.toRadians(270))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    d.blueArm.setPosition(0);
                    d.blackArm.setPosition(0);
                    d.rotateServo.setPosition(0);
                    d.blackLift.setTargetPosition(0);
                    d.blueLift.setTargetPosition(0);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                })
                .build(); // If middle

        TrajectorySequence leftSideParkRight = d.trajectorySequenceBuilder(leftSide.end())
                .splineToSplineHeading(new Pose2d(-33, -31, Math.toRadians(270)), Math.toRadians(270))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    d.blueArm.setPosition(0);
                    d.blackArm.setPosition(0);
                    d.rotateServo.setPosition(0);
                    d.blackLift.setTargetPosition(0);
                    d.blueLift.setTargetPosition(0);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                })
                .lineToConstantHeading(new Vector2d(-9, -31))
                .build(); // If right

        array = new TrajectorySequence[] {leftSide, leftSideCones, leftSideParkLeft, leftSideParkMiddle, leftSideParkRight};
        return array;
    }

    private TrajectorySequence[] buildRightSide() {
        TrajectorySequence rightSide = d.trajectorySequenceBuilder(d.getPoseEstimate())
                .splineToSplineHeading(new Pose2d(34, -47, Math.toRadians(0)), Math.toRadians(90))

                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    d.blueGripper.setPosition(0.8);
                    d.blackGripper.setPosition(0.8);
                })
/*                .UNSTABLE_addTemporalMarkerOffset(-0.6, () -> {
                    d.blackLift.setTargetPosition(topJunction);
                    d.blueLift.setTargetPosition(topJunction);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                    d.blueArm.setPosition(1);
                    d.blackArm.setPosition(1);
                    d.rotateServo.setPosition(1);
                })*/
                .splineToSplineHeading(new Pose2d(34, -18, Math.toRadians(0)), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(29, -5, Math.toRadians(315)), Math.toRadians(135))
                .waitSeconds(0.2)
                .UNSTABLE_addDisplacementMarkerOffset(-0.1, () -> {
                    d.blackGripper.setPosition(0);
                    d.blueGripper.setPosition(0);
                })
                .build(); // 9 x 8 y

        TrajectorySequence rightSideCones = d.trajectorySequenceBuilder(rightSide.end())
                // Grab first cone from stack
                .splineToSplineHeading(new Pose2d(47, -8, Math.toRadians(0)), Math.toRadians(0))
/*                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    d.blueArm.setPosition(0);
                    d.blackArm.setPosition(0);
                    d.rotateServo.setPosition(0);
                    d.blackLift.setTargetPosition(liftLevel);
                    d.blueLift.setTargetPosition(liftLevel);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                })*/
                .splineToSplineHeading(new Pose2d(57, -8, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(-0.25, () -> {
                    d.blackGripper.setPosition(0.8);
                    d.blueGripper.setPosition(0.8);
                })
                // Place first cone
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(47, -8, Math.toRadians(0)), Math.toRadians(180))
/*                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    d.blueArm.setPosition(1);
                    d.blackArm.setPosition(1);
                    d.rotateServo.setPosition(1);
                    d.blackLift.setTargetPosition(topJunction);
                    d.blueLift.setTargetPosition(topJunction);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                })*/
                .splineToSplineHeading(new Pose2d(29, -5, Math.toRadians(315)), Math.toRadians(135))
                .waitSeconds(0.2)
                .UNSTABLE_addDisplacementMarkerOffset(-0.2, () -> {
                    d.blackGripper.setPosition(0);
                    d.blueGripper.setPosition(0);
                })
                .setReversed(false)
                .build();

        TrajectorySequence rightSideParkRight = d.trajectorySequenceBuilder(rightSide.end())
                .splineToSplineHeading(new Pose2d(34, -31.5, Math.toRadians(270)), Math.toRadians(270))
/*                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    d.blueArm.setPosition(0);
                    d.blackArm.setPosition(0);
                    d.rotateServo.setPosition(0);
                    d.blackLift.setTargetPosition(0);
                    d.blueLift.setTargetPosition(0);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                })*/
                .lineToLinearHeading(new Pose2d(58, -31.5, Math.toRadians(270)))
                .build(); // If left

        TrajectorySequence rightSideParkMiddle = d.trajectorySequenceBuilder(rightSide.end())
                .splineToSplineHeading(new Pose2d(34, -31.5, Math.toRadians(270)), Math.toRadians(270))
/*                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    d.blueArm.setPosition(0);
                    d.blackArm.setPosition(0);
                    d.rotateServo.setPosition(0);
                    d.blackLift.setTargetPosition(0);
                    d.blueLift.setTargetPosition(0);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                })*/
                .build(); // If middle

        TrajectorySequence rightSideParkLeft = d.trajectorySequenceBuilder(rightSide.end())
                .splineToSplineHeading(new Pose2d(34, -31.5, Math.toRadians(270)), Math.toRadians(270))
/*                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    d.blueArm.setPosition(0);
                    d.blackArm.setPosition(0);
                    d.rotateServo.setPosition(0);
                    d.blackLift.setTargetPosition(0);
                    d.blueLift.setTargetPosition(0);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                })*/
                .lineToLinearHeading(new Pose2d(12, -31.5, Math.toRadians(270)))
                .build(); // If left

        return new TrajectorySequence[] {rightSide, rightSideCones, rightSideParkLeft, rightSideParkMiddle, rightSideParkRight};


    }

    @Config
    public class ColorDetectionPipeline extends OpenCvPipeline {
        //Mat objects to hold the original image,
        // the filtered image, and the region of interest
        private final Mat hsvMat = new Mat();
        private final Mat mask = new Mat();
        private final Mat filtered = new Mat();
        private final Mat filteredG = new Mat();

        // Declare the rectangle to be used for processing
        Rect rect = new Rect(rectX, rectY, rectWidth, rectHeight);

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
                color = WorldsConfig.colors.green;
            } else if (yellowCount > greenCount && yellowCount > purpleCount) {
                color = WorldsConfig.colors.yellow;
            } else if (purpleCount > greenCount && purpleCount > yellowCount) {
                color = WorldsConfig.colors.purple;
            } else {
                color = WorldsConfig.colors.None;
            }

            telemetry.addData("Color", color);
            telemetry.addData("Side", side);
            telemetry.addData("Cone count", coneCounter);
            telemetry.addData("Robot ready? ", ready);
            telemetry.addData("BLack cam FPS", blackCam.getFps());
            telemetry.addData("Blue cam FPS", blueCam.getFps());
            telemetry.update();
            return input;
        }
    }
}
/*            switch (side) {
        case right:
            d.followTrajectorySequence(rightSide);
            if (coneCounter >= 1) d.followTrajectorySequence(rightSideCones);
            blueCam.stopStreaming();
            switch (color) {
                case green:
                    d.followTrajectorySequence(rightSideParkLeft);
                    break;
                case yellow:
                    d.followTrajectorySequence(rightSideParkMiddle);
                    break;
                case purple:
                    d.followTrajectorySequence(rightSideParkRight);
                    break;
            }
            break;

        case left:
            blackCam.stopStreaming();
            d.followTrajectorySequence(leftSide);
            if (coneCounter >= 1) d.followTrajectorySequence(leftSideCones);
            switch (color) {
                case green:
                    d.followTrajectorySequence(leftSideParkLeft);
                    break;
                case yellow:
                    d.followTrajectorySequence(leftSideParkMiddle);
                    break;
                case purple:
                    d.followTrajectorySequence(leftSideParkRight);
                    break;
            }
            break;
    }*/
