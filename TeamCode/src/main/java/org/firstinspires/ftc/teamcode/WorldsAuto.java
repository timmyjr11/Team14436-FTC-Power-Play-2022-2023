package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
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

    int liftLevel = 700;
    int topJunction = 1500;

    double lowerGripperPos = 0;

    int leftForward = 0;
    int leftSideLevel = 0;
    int rightForward = 0;
    int rightSideLevel = 0;

    int coneCounter;

    WorldsConfig.side side = WorldsConfig.side.tbd;
    WorldsConfig.type type = WorldsConfig.type.tbd;

    ArrayList<Boolean> booleanArray = new ArrayList<>();
    TrajectorySequence[] singleJunctionArray;
    TrajectorySequence[] multiJunctionArray;

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

    public static double upperPurpleH = 160;
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

        d.blueArm.setPosition(lowerGripperPos);
        d.blackArm.setPosition(lowerGripperPos);
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
                side = WorldsConfig.side.left;
                rectY = 45;
                blackCam.setPipeline(new ColorDetectionPipeline());
                blackCam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
                dash.startCameraStream(blackCam, 30);
                break;
            } else if (gamepad1.dpad_right) {
                rectY = 95;
                side = WorldsConfig.side.right;
                blueCam.setPipeline(new ColorDetectionPipeline());
                blueCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                dash.startCameraStream(blueCam, 30);
                break;
            }
            if (isStopRequested()) return;
        }

        while (true) {
            telemetry.addLine("What type of auto would you like to run?");
            telemetry.addLine("Press up on D-Pad for Single junction");
            telemetry.addLine("Press down on D-Pad for multi-junction");
            telemetry.update();

            if (gamepad1.dpad_up) {
                type = WorldsConfig.type.singleJunction;
                break;
            } else if (gamepad1.dpad_down) {
                type = WorldsConfig.type.multiJunction;
                break;
            }
            if (isStopRequested()) return;
        }

        if (type == WorldsConfig.type.singleJunction) {
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
            if (side == WorldsConfig.side.left) {
                d.setPoseEstimate(WorldsConfig.leftAutoSingle);
            } else {
                d.setPoseEstimate(WorldsConfig.rightAutoSingle);
            }
            singleJunctionArray = (side == WorldsConfig.side.left) ? buildLeftSideSingle() : buildRightSideSingle();
        } else {
            if (side == WorldsConfig.side.left) {
                d.setPoseEstimate(WorldsConfig.leftAutoMulti);
            } else {
                d.setPoseEstimate(WorldsConfig.rightAutoMulti);
            }
            multiJunctionArray = (side == WorldsConfig.side.left) ? buildLeftSideMulti() : buildRightSideMulti();
        }

        ready = true;

        while (!isStarted()) if (isStopRequested()) return;

        if (side == WorldsConfig.side.left) {
            blackCam.stopStreaming();
        } else {
            blueCam.stopStreaming();
        }

        if (type == WorldsConfig.type.singleJunction) {
            d.followTrajectorySequence(singleJunctionArray[0]);

            if (coneCounter >= 1) d.followTrajectorySequence(singleJunctionArray[1]);
            liftLevel -= 150;
            if (coneCounter >= 2) d.followTrajectorySequence(singleJunctionArray[2]);
            liftLevel -= 150;
            if (coneCounter >= 3) d.followTrajectorySequence(singleJunctionArray[3]);
            liftLevel -= 150;
            if (coneCounter >= 4) d.followTrajectorySequence(singleJunctionArray[4]);
            liftLevel -= 150;
            if (coneCounter == 5) d.followTrajectorySequence(singleJunctionArray[5]);

            switch (color) {
                case green:
                    d.followTrajectorySequence(singleJunctionArray[6]);
                    break;
                case yellow:
                    d.followTrajectorySequence(singleJunctionArray[7]);
                    break;
                case purple:
                    d.followTrajectorySequence(singleJunctionArray[8]);
                    break;
            }
        } else {
            d.followTrajectorySequence(multiJunctionArray[0]);
            switch (color) {
                case green:
                    d.followTrajectorySequence(multiJunctionArray[1]);
                    break;
                case yellow:
                    d.followTrajectorySequence(multiJunctionArray[2]);
                    break;
                case purple:
                    d.followTrajectorySequence(multiJunctionArray[3]);
                    break;
            }
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

    private TrajectorySequence[] buildLeftSideSingle() {

        TrajectorySequence leftSide = d.trajectorySequenceBuilder(d.getPoseEstimate())
                .splineToSplineHeading(new Pose2d(-33, -47, Math.toRadians(180)), Math.toRadians(90))

                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    d.blueGripper.setPosition(1);
                    d.blackGripper.setPosition(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    d.blackLift.setTargetPosition(400);
                    d.blueLift.setTargetPosition(400);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    d.blackLift.setTargetPosition(topJunction);
                    d.blueLift.setTargetPosition(topJunction);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                    d.blueArm.setPosition(0.83);
                    d.blackArm.setPosition(0.83);
                    d.rotateServo.setPosition(1);
                })
                .splineToSplineHeading(new Pose2d(-33, -20, Math.toRadians(180)), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-27.8, -4.75, Math.toRadians(225)), Math.toRadians(45))
                .waitSeconds(0.2)
                .UNSTABLE_addDisplacementMarkerOffset(-0.1, () -> {
                    d.blackGripper.setPosition(0);
                    d.blueGripper.setPosition(0);
                })
                .build(); // 9 x 8 y


        TrajectorySequence leftSideCones1 = d.trajectorySequenceBuilder(leftSide.end())
                // Grab first cone from stack
                .splineToSplineHeading(new Pose2d(-47, -8, Math.toRadians(180)), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    d.blueArm.setPosition(lowerGripperPos);
                    d.blackArm.setPosition(lowerGripperPos);
                    d.rotateServo.setPosition(0);
                    d.blackLift.setTargetPosition(liftLevel);
                    d.blueLift.setTargetPosition(liftLevel);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                })

                .splineToSplineHeading(new Pose2d(-55.5, -8, Math.toRadians(180)), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> {
                    d.blackGripper.setPosition(1);
                    d.blueGripper.setPosition(1);
                })
                // Place first cone
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-47, -8, Math.toRadians(180)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    d.blackLift.setTargetPosition(topJunction);
                    d.blueLift.setTargetPosition(topJunction);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> {
                    d.blueArm.setPosition(0.83);
                    d.blackArm.setPosition(0.83);
                    d.rotateServo.setPosition(1);
                })
                .splineToSplineHeading(new Pose2d(-28, -4.5, Math.toRadians(225)), Math.toRadians(45))
                .waitSeconds(0.1)
                .UNSTABLE_addDisplacementMarkerOffset(-0.1, () -> {
                    d.blackGripper.setPosition(0);
                    d.blueGripper.setPosition(0);
                })
                .setReversed(false)
                .build();


        TrajectorySequence leftSideCones2 = d.trajectorySequenceBuilder(leftSideCones1.end())
                // Grab first cone from stack
                .splineToSplineHeading(new Pose2d(-47, -8.5, Math.toRadians(180)), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    d.blueArm.setPosition(lowerGripperPos);
                    d.blackArm.setPosition(lowerGripperPos);
                    d.rotateServo.setPosition(0);
                    d.blackLift.setTargetPosition(liftLevel);
                    d.blueLift.setTargetPosition(liftLevel);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                })

                .splineToSplineHeading(new Pose2d(-55.25, -8.5, Math.toRadians(180)), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> {
                    d.blackGripper.setPosition(1);
                    d.blueGripper.setPosition(1);
                })
                // Place first cone
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-47, -8.5, Math.toRadians(180)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    d.blackLift.setTargetPosition(topJunction);
                    d.blueLift.setTargetPosition(topJunction);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> {
                    d.blueArm.setPosition(0.83);
                    d.blackArm.setPosition(0.83);
                    d.rotateServo.setPosition(1);
                })
                .splineToSplineHeading(new Pose2d(-28, -4.5, Math.toRadians(225)), Math.toRadians(45))
                .waitSeconds(0.1)
                .UNSTABLE_addDisplacementMarkerOffset(-0.1, () -> {
                    d.blackGripper.setPosition(0);
                    d.blueGripper.setPosition(0);
                })
                .setReversed(false)
                .build();

        TrajectorySequence leftSideCones3 = d.trajectorySequenceBuilder(leftSideCones2.end())
                // Grab first cone from stack
                .splineToSplineHeading(new Pose2d(-47, -8.5, Math.toRadians(180)), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    d.blueArm.setPosition(lowerGripperPos);
                    d.blackArm.setPosition(lowerGripperPos);
                    d.rotateServo.setPosition(0);
                    d.blackLift.setTargetPosition(liftLevel);
                    d.blueLift.setTargetPosition(liftLevel);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                })

                .splineToSplineHeading(new Pose2d(-55.25, -8.5, Math.toRadians(180)), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> {
                    d.blackGripper.setPosition(1);
                    d.blueGripper.setPosition(1);
                })
                // Place first cone
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-47, -8.5, Math.toRadians(180)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    d.blackLift.setTargetPosition(topJunction);
                    d.blueLift.setTargetPosition(topJunction);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> {
                    d.blueArm.setPosition(0.83);
                    d.blackArm.setPosition(0.83);
                    d.rotateServo.setPosition(1);
                })
                .splineToSplineHeading(new Pose2d(-28, -4.75, Math.toRadians(225)), Math.toRadians(45))
                .waitSeconds(0.1)
                .UNSTABLE_addDisplacementMarkerOffset(-0.1, () -> {
                    d.blackGripper.setPosition(0);
                    d.blueGripper.setPosition(0);
                })
                .setReversed(false)
                .build();


        TrajectorySequence leftSideCones4 = d.trajectorySequenceBuilder(leftSideCones3.end())
                // Grab first cone from stack
                .splineToSplineHeading(new Pose2d(-47, -8.5, Math.toRadians(180)), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    d.blueArm.setPosition(lowerGripperPos);
                    d.blackArm.setPosition(lowerGripperPos);
                    d.rotateServo.setPosition(0);
                    d.blackLift.setTargetPosition(liftLevel);
                    d.blueLift.setTargetPosition(liftLevel);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                })

                .splineToSplineHeading(new Pose2d(-55.25, -8.5, Math.toRadians(180)), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> {
                    d.blackGripper.setPosition(1);
                    d.blueGripper.setPosition(1);
                })
                // Place first cone
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-47, -8.5, Math.toRadians(180)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    d.blackLift.setTargetPosition(topJunction);
                    d.blueLift.setTargetPosition(topJunction);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> {
                    d.blueArm.setPosition(0.83);
                    d.blackArm.setPosition(0.83);
                    d.rotateServo.setPosition(1);
                })
                .splineToSplineHeading(new Pose2d(-28, -4.75, Math.toRadians(225)), Math.toRadians(45))
                .waitSeconds(0.1)
                .UNSTABLE_addDisplacementMarkerOffset(-0.1, () -> {
                    d.blackGripper.setPosition(0);
                    d.blueGripper.setPosition(0);
                })
                .setReversed(false)
                .build();

        TrajectorySequence leftSideCones5 = d.trajectorySequenceBuilder(leftSideCones4.end())
                // Grab first cone from stack
                .splineToSplineHeading(new Pose2d(-47, -8.5, Math.toRadians(180)), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    d.blueArm.setPosition(lowerGripperPos);
                    d.blackArm.setPosition(lowerGripperPos);
                    d.rotateServo.setPosition(0);
                    d.blackLift.setTargetPosition(liftLevel);
                    d.blueLift.setTargetPosition(liftLevel);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                })

                .splineToSplineHeading(new Pose2d(-55.25, -8.5, Math.toRadians(180)), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> {
                    d.blackGripper.setPosition(1);
                    d.blueGripper.setPosition(1);
                })
                // Place first cone
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-47, -8.5, Math.toRadians(180)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    d.blackLift.setTargetPosition(topJunction);
                    d.blueLift.setTargetPosition(topJunction);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> {
                    d.blueArm.setPosition(0.83);
                    d.blackArm.setPosition(0.83);
                    d.rotateServo.setPosition(1);
                })
                .splineToSplineHeading(new Pose2d(-28, -4.75, Math.toRadians(225)), Math.toRadians(45))
                .waitSeconds(0.1)
                .UNSTABLE_addDisplacementMarkerOffset(-0.1, () -> {
                    d.blackGripper.setPosition(0);
                    d.blueGripper.setPosition(0);
                })
                .setReversed(false)
                .build();


        TrajectorySequence leftSideParkLeft = d.trajectorySequenceBuilder(leftSide.end())
                .splineToSplineHeading(new Pose2d(-56, -8.5, Math.toRadians(270)), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    d.blueArm.setPosition(lowerGripperPos);
                    d.blackArm.setPosition(lowerGripperPos);
                    d.rotateServo.setPosition(0);
                    d.blackLift.setTargetPosition(0);
                    d.blueLift.setTargetPosition(0);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                })
                //.lineToLinearHeading(new Pose2d(-56, -8, Math.toRadians(270)))
                .waitSeconds(0.5)
                .build(); // If left

        TrajectorySequence leftSideParkMiddle = d.trajectorySequenceBuilder(leftSide.end())
                .lineToSplineHeading(new Pose2d(-32, -11, Math.toRadians(270)))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    d.blueArm.setPosition(lowerGripperPos);
                    d.blackArm.setPosition(lowerGripperPos);
                    d.rotateServo.setPosition(0);
                    d.blackLift.setTargetPosition(0);
                    d.blueLift.setTargetPosition(0);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                })
                .waitSeconds(0.5)
                .build(); // If middle

        TrajectorySequence leftSideParkRight = d.trajectorySequenceBuilder(leftSide.end())
                .splineToSplineHeading(new Pose2d(-9, -10, Math.toRadians(270)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    d.blueArm.setPosition(lowerGripperPos);
                    d.blackArm.setPosition(lowerGripperPos);
                    d.rotateServo.setPosition(0);
                    d.blackLift.setTargetPosition(0);
                    d.blueLift.setTargetPosition(0);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                })
                .waitSeconds(0.5)
                .build(); // If right

        return new TrajectorySequence[] {leftSide, leftSideCones1 ,leftSideCones2, leftSideCones3, leftSideCones4, leftSideCones5,
                leftSideParkLeft, leftSideParkMiddle, leftSideParkRight};
    }

    private TrajectorySequence[] buildLeftSideMulti() {

        TrajectorySequence leftSide = d.trajectorySequenceBuilder(d.getPoseEstimate())
                // Place the first cone
                .UNSTABLE_addTemporalMarkerOffset(0.05, () ->{
                    d.blueGripper.setPosition(1);
                    d.blackGripper.setPosition(1);
                })
                .splineToSplineHeading(new Pose2d(-34, -40, Math.toRadians(90)), Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(-0.6, () -> {
                    d.blackLift.setTargetPosition(1800);
                    d.blueLift.setTargetPosition(1800);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                })
                .splineToSplineHeading(new Pose2d(-38, -24, Math.toRadians(135)), Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> {
                    d.blackGripper.setPosition(0);
                    d.blueGripper.setPosition(0);
                })
                .waitSeconds(0.3)

                // Go to grab first cone
                .lineToSplineHeading(new Pose2d(-34, -35, Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(-34, -8, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(-0.7, () -> {
                    d.blackLift.setTargetPosition(liftLevel - 50);
                    d.blueLift.setTargetPosition(liftLevel - 50);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                    d.blackArm.setPosition(lowerGripperPos);
                    d.blueArm.setPosition(lowerGripperPos);
                })
                .lineToSplineHeading(new Pose2d(-57, -8, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> {
                    d.blueGripper.setPosition(1);
                    d.blackGripper.setPosition(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    d.blackLift.setTargetPosition(topJunction);
                    d.blueLift.setTargetPosition(topJunction);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                })
                .waitSeconds(0.2)

                // Go place first cone
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-40, -8, Math.toRadians(180)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> {
                    d.blackLift.setTargetPosition(250);
                    d.blueLift.setTargetPosition(250);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                    d.blueArm.setPosition(0.83);
                    d.blackArm.setPosition(0.83);
                    d.rotateServo.setPosition(1);
                })
                .splineToSplineHeading(new Pose2d(-30, -11, Math.toRadians(135)), Math.toRadians(315))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    d.blueGripper.setPosition(0);
                    d.blackGripper.setPosition(0);
                })
                .waitSeconds(0.4)

                // Go grab second cone
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(-40, -7, Math.toRadians(180)), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-0.7, () -> {
                    d.blackLift.setTargetPosition(liftLevel - 200);
                    d.blueLift.setTargetPosition(liftLevel - 200);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                    d.rotateServo.setPosition(0);
                    d.blackArm.setPosition(lowerGripperPos);
                    d.blueArm.setPosition(lowerGripperPos);
                })
                .splineToSplineHeading(new Pose2d(-57, -7, Math.toRadians(180)), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> {
                    d.blueGripper.setPosition(1);
                    d.blackGripper.setPosition(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    d.blackLift.setTargetPosition(topJunction);
                    d.blueLift.setTargetPosition(topJunction);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                    d.blueArm.setPosition(lowerGripperPos);
                    d.blackArm.setPosition(lowerGripperPos);
                    d.rotateServo.setPosition(0);
                })
                .waitSeconds(0.2)

                // Go place second cone
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-20, -7, Math.toRadians(180)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    d.blackLift.setTargetPosition(topJunction);
                    d.blueLift.setTargetPosition(topJunction);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                    d.blueArm.setPosition(0.83);
                    d.blackArm.setPosition(0.83);
                    d.rotateServo.setPosition(1);
                })
                .splineToSplineHeading(new Pose2d(-5, -8, Math.toRadians(135)), Math.toRadians(315))
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    d.blueGripper.setPosition(0);
                    d.blackGripper.setPosition(0);
                })
                .waitSeconds(0.5)
                .build();

        TrajectorySequence leftSideParkLeft = d.trajectorySequenceBuilder(leftSide.end())
                .lineToSplineHeading(new Pose2d(-8, -7, Math.toRadians(270)))
                .lineToSplineHeading(new Pose2d(-56, -7, Math.toRadians(270)))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    d.blueArm.setPosition(lowerGripperPos);
                    d.blackArm.setPosition(lowerGripperPos);
                    d.rotateServo.setPosition(0);
                    d.blackLift.setTargetPosition(0);
                    d.blueLift.setTargetPosition(0);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                })
                .waitSeconds(0.5)
                .build(); // If left

        TrajectorySequence leftSideParkMiddle = d.trajectorySequenceBuilder(leftSide.end())
                .splineToSplineHeading(new Pose2d(-32, -7, Math.toRadians(270)), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    d.blueArm.setPosition(lowerGripperPos);
                    d.blackArm.setPosition(lowerGripperPos);
                    d.rotateServo.setPosition(0);
                    d.blackLift.setTargetPosition(0);
                    d.blueLift.setTargetPosition(0);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                })
                .waitSeconds(0.5)
                .build(); // If middle

        TrajectorySequence leftSideParkRight = d.trajectorySequenceBuilder(leftSide.end())
                .splineToSplineHeading(new Pose2d(-9, -7, Math.toRadians(270)), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    d.blueArm.setPosition(lowerGripperPos);
                    d.blackArm.setPosition(lowerGripperPos);
                    d.rotateServo.setPosition(0);
                    d.blackLift.setTargetPosition(0);
                    d.blueLift.setTargetPosition(0);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                })
                .waitSeconds(0.5)
                .build(); // If right

        return new TrajectorySequence[] {leftSide, leftSideParkLeft, leftSideParkMiddle, leftSideParkRight};
    }

    private TrajectorySequence[] buildRightSideSingle() {

        TrajectorySequence rightSide = d.trajectorySequenceBuilder(d.getPoseEstimate())
                .splineToSplineHeading(new Pose2d(35, -47, Math.toRadians(0)), Math.toRadians(90))

                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    d.blueGripper.setPosition(1);
                    d.blackGripper.setPosition(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    d.blackLift.setTargetPosition(400);
                    d.blueLift.setTargetPosition(400);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    d.blackLift.setTargetPosition(topJunction + 100);
                    d.blueLift.setTargetPosition(topJunction + 100);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                    d.blueArm.setPosition(0.83);
                    d.blackArm.setPosition(0.83);
                    d.rotateServo.setPosition(1);
                })
                .splineToSplineHeading(new Pose2d(35, -20, Math.toRadians(0)), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(32.1, -4.5, Math.toRadians(315)), Math.toRadians(135))
                .waitSeconds(0.5)
                .UNSTABLE_addDisplacementMarkerOffset(-0.05, () -> {
                    d.blackGripper.setPosition(0);
                    d.blueGripper.setPosition(0);
                })
                .build(); // 9 x 8 y


        TrajectorySequence rightSideCones1 = d.trajectorySequenceBuilder(rightSide.end())
                // Grab first cone from stack
                .splineToSplineHeading(new Pose2d(47, -9, Math.toRadians(0)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-2.5, () -> {
                    d.blueArm.setPosition(lowerGripperPos);
                    d.blackArm.setPosition(lowerGripperPos);
                    d.rotateServo.setPosition(0);
                    d.blackLift.setTargetPosition(liftLevel);
                    d.blueLift.setTargetPosition(liftLevel);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                })

                .splineToSplineHeading(new Pose2d(59, -9, Math.toRadians(0)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> {
                    d.blackGripper.setPosition(1);
                    d.blueGripper.setPosition(1);
                })
                // Place first cone
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(47, -9, Math.toRadians(0)), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-1.25, () -> {
                    d.blackLift.setTargetPosition(topJunction);
                    d.blueLift.setTargetPosition(topJunction);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.6, () -> {
                    d.blueArm.setPosition(0.83);
                    d.blackArm.setPosition(0.83);
                    d.rotateServo.setPosition(1);
                })
                .splineToSplineHeading(new Pose2d(31.5, -4.75, Math.toRadians(315)), Math.toRadians(135))
                .waitSeconds(0.2)
                .UNSTABLE_addDisplacementMarkerOffset(-0.05, () -> {
                    d.blackGripper.setPosition(0);
                    d.blueGripper.setPosition(0);
                })
                .setReversed(false)
                .build();


        TrajectorySequence rightSideCones2 = d.trajectorySequenceBuilder(rightSideCones1.end())
                // Grab first cone from stack
                .splineToSplineHeading(new Pose2d(47, -9, Math.toRadians(0)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    d.blueArm.setPosition(lowerGripperPos);
                    d.blackArm.setPosition(lowerGripperPos);
                    d.rotateServo.setPosition(0);
                    d.blackLift.setTargetPosition(liftLevel);
                    d.blueLift.setTargetPosition(liftLevel);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                })

                .splineToSplineHeading(new Pose2d(59, -9, Math.toRadians(0)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> {
                    d.blackGripper.setPosition(1);
                    d.blueGripper.setPosition(1);
                })
                // Place first cone
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(47, -9, Math.toRadians(0)), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-1.25, () -> {
                    d.blackLift.setTargetPosition(topJunction);
                    d.blueLift.setTargetPosition(topJunction);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.6, () -> {
                    d.blueArm.setPosition(0.83);
                    d.blackArm.setPosition(0.83);
                    d.rotateServo.setPosition(1);
                })
                .splineToSplineHeading(new Pose2d(31.5, -6, Math.toRadians(315)), Math.toRadians(135))
                .waitSeconds(0.2)
                .UNSTABLE_addDisplacementMarkerOffset(-0.05, () -> {
                    d.blackGripper.setPosition(0);
                    d.blueGripper.setPosition(0);
                })
                .setReversed(false)
                .build();

        TrajectorySequence rightSideCones3 = d.trajectorySequenceBuilder(rightSideCones2.end())
                // Grab first cone from stack
                .splineToSplineHeading(new Pose2d(47, -10, Math.toRadians(0)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    d.blueArm.setPosition(lowerGripperPos);
                    d.blackArm.setPosition(lowerGripperPos);
                    d.rotateServo.setPosition(0);
                    d.blackLift.setTargetPosition(liftLevel);
                    d.blueLift.setTargetPosition(liftLevel);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                })

                .splineToSplineHeading(new Pose2d(59, -10, Math.toRadians(0)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> {
                    d.blackGripper.setPosition(1);
                    d.blueGripper.setPosition(1);
                })
                // Place first cone
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(47, -10, Math.toRadians(0)), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-1.25, () -> {
                    d.blackLift.setTargetPosition(topJunction);
                    d.blueLift.setTargetPosition(topJunction);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.6, () -> {
                    d.blueArm.setPosition(0.83);
                    d.blackArm.setPosition(0.83);
                    d.rotateServo.setPosition(1);
                })
                .splineToSplineHeading(new Pose2d(31.5, -6.75, Math.toRadians(315)), Math.toRadians(135))
                .waitSeconds(0.2)
                .UNSTABLE_addDisplacementMarkerOffset(-0.05, () -> {
                    d.blackGripper.setPosition(0);
                    d.blueGripper.setPosition(0);
                })
                .setReversed(false)
                .build();


        TrajectorySequence rightSideCones4 = d.trajectorySequenceBuilder(rightSideCones3.end())
                // Grab first cone from stack
                .splineToSplineHeading(new Pose2d(47, -11, Math.toRadians(0)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    d.blueArm.setPosition(lowerGripperPos);
                    d.blackArm.setPosition(lowerGripperPos);
                    d.rotateServo.setPosition(0);
                    d.blackLift.setTargetPosition(liftLevel);
                    d.blueLift.setTargetPosition(liftLevel);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                })

                .splineToSplineHeading(new Pose2d(59, -11, Math.toRadians(0)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> {
                    d.blackGripper.setPosition(1);
                    d.blueGripper.setPosition(1);
                })
                // Place first cone
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(47, -11, Math.toRadians(0)), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-1.25, () -> {
                    d.blackLift.setTargetPosition(topJunction);
                    d.blueLift.setTargetPosition(topJunction);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.6, () -> {
                    d.blueArm.setPosition(0.83);
                    d.blackArm.setPosition(0.83);
                    d.rotateServo.setPosition(1);
                })
                .splineToSplineHeading(new Pose2d(31.5, -7, Math.toRadians(315)), Math.toRadians(135))
                .waitSeconds(0.2)
                .UNSTABLE_addDisplacementMarkerOffset(-0.05, () -> {
                    d.blackGripper.setPosition(0);
                    d.blueGripper.setPosition(0);
                })
                .setReversed(false)
                .build();

        TrajectorySequence rightSideCones5 = d.trajectorySequenceBuilder(rightSideCones4.end())
                // Grab first cone from stack
                .splineToSplineHeading(new Pose2d(47, -11.5, Math.toRadians(0)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    d.blueArm.setPosition(lowerGripperPos);
                    d.blackArm.setPosition(lowerGripperPos);
                    d.rotateServo.setPosition(0);
                    d.blackLift.setTargetPosition(liftLevel);
                    d.blueLift.setTargetPosition(liftLevel);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                })

                .splineToSplineHeading(new Pose2d(59, -11.5, Math.toRadians(0)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> {
                    d.blackGripper.setPosition(1);
                    d.blueGripper.setPosition(1);
                })
                // Place first cone
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(47, -11.5, Math.toRadians(0)), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-1.25, () -> {
                    d.blackLift.setTargetPosition(topJunction);
                    d.blueLift.setTargetPosition(topJunction);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.6, () -> {
                    d.blueArm.setPosition(0.83);
                    d.blackArm.setPosition(0.83);
                    d.rotateServo.setPosition(1);
                })
                .splineToSplineHeading(new Pose2d(31.5, -7, Math.toRadians(315)), Math.toRadians(135))
                .waitSeconds(0.2)
                .UNSTABLE_addDisplacementMarkerOffset(-0.1, () -> {
                    d.blackGripper.setPosition(0);
                    d.blueGripper.setPosition(0);
                })
                .setReversed(false)
                .build();


        TrajectorySequence rightSideParkRight = d.trajectorySequenceBuilder(rightSide.end())
                .splineToSplineHeading(new Pose2d(56.5, -11.5, Math.toRadians(270)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    d.blueArm.setPosition(lowerGripperPos);
                    d.blackArm.setPosition(lowerGripperPos);
                    d.rotateServo.setPosition(0);
                    d.blackLift.setTargetPosition(0);
                    d.blueLift.setTargetPosition(0);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                })
                .waitSeconds(0.5)
                .build(); // If left

        TrajectorySequence rightSideParkMiddle = d.trajectorySequenceBuilder(rightSide.end())
                .lineToSplineHeading(new Pose2d(33, -11.5, Math.toRadians(270)))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    d.blueArm.setPosition(lowerGripperPos);
                    d.blackArm.setPosition(lowerGripperPos);
                    d.rotateServo.setPosition(0);
                    d.blackLift.setTargetPosition(0);
                    d.blueLift.setTargetPosition(0);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                })
                .waitSeconds(0.5)
                .build(); // If middle

        TrajectorySequence rightSideParkLeft = d.trajectorySequenceBuilder(rightSide.end())
                .splineToSplineHeading(new Pose2d(15, -11.5, Math.toRadians(270)), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    d.blueArm.setPosition(lowerGripperPos);
                    d.blackArm.setPosition(lowerGripperPos);
                    d.rotateServo.setPosition(0);
                    d.blackLift.setTargetPosition(0);
                    d.blueLift.setTargetPosition(0);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                })
                .waitSeconds(0.5)
                .build(); // If right

         return new TrajectorySequence[] {rightSide, rightSideCones1 ,rightSideCones2, rightSideCones3, rightSideCones4, rightSideCones5,
                rightSideParkLeft, rightSideParkMiddle, rightSideParkRight};
    }

    private TrajectorySequence[] buildRightSideMulti() {

        TrajectorySequence leftSide = d.trajectorySequenceBuilder(d.getPoseEstimate())
                // Place the first cone
                .UNSTABLE_addTemporalMarkerOffset(0.05, () ->{
                    d.blueGripper.setPosition(1);
                    d.blackGripper.setPosition(1);
                })
                .splineToSplineHeading(new Pose2d(34, -40, Math.toRadians(90)), Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(-0.6, () -> {
                    d.blackLift.setTargetPosition(2000);
                    d.blueLift.setTargetPosition(2000);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                })
                .splineToSplineHeading(new Pose2d(38, -24, Math.toRadians(45)), Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> {
                    d.blackGripper.setPosition(0);
                    d.blueGripper.setPosition(0);
                })
                .waitSeconds(0.3)

                // Go to grab first cone
                .lineToSplineHeading(new Pose2d(34, -35, Math.toRadians(0)))
                .lineToSplineHeading(new Pose2d(34, -8, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(-0.7, () -> {
                    d.blackLift.setTargetPosition(liftLevel - 50);
                    d.blueLift.setTargetPosition(liftLevel - 50);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                    d.blackArm.setPosition(lowerGripperPos);
                    d.blueArm.setPosition(lowerGripperPos);
                })
                .lineToSplineHeading(new Pose2d(57, -8, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> {
                    d.blueGripper.setPosition(1);
                    d.blackGripper.setPosition(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    d.blackLift.setTargetPosition(topJunction);
                    d.blueLift.setTargetPosition(topJunction);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                })
                .waitSeconds(0.2)

                // Go place first cone
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(40, -8, Math.toRadians(0)), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> {
                    d.blackLift.setTargetPosition(250);
                    d.blueLift.setTargetPosition(250);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                    d.blueArm.setPosition(0.83);
                    d.blackArm.setPosition(0.83);
                    d.rotateServo.setPosition(1);
                })
                .splineToSplineHeading(new Pose2d(30, -11, Math.toRadians(45)), Math.toRadians(225)) //TODO: tangent
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    d.blueGripper.setPosition(0);
                    d.blackGripper.setPosition(0);
                })
                .waitSeconds(0.4)

                // Go grab second cone
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(40, -7, Math.toRadians(0)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-0.7, () -> {
                    d.blackLift.setTargetPosition(liftLevel - 200);
                    d.blueLift.setTargetPosition(liftLevel - 200);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                    d.rotateServo.setPosition(0);
                    d.blackArm.setPosition(lowerGripperPos);
                    d.blueArm.setPosition(lowerGripperPos);
                })
                .splineToSplineHeading(new Pose2d(57, -7, Math.toRadians(0)), Math.toRadians(0))// TODO CONTINUE HERE.
                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> {
                    d.blueGripper.setPosition(1);
                    d.blackGripper.setPosition(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    d.blackLift.setTargetPosition(topJunction);
                    d.blueLift.setTargetPosition(topJunction);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                    d.blueArm.setPosition(lowerGripperPos);
                    d.blackArm.setPosition(lowerGripperPos);
                    d.rotateServo.setPosition(0);
                })
                .waitSeconds(0.2)

                // Go place second cone
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(20, -7, Math.toRadians(0)), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    d.blackLift.setTargetPosition(topJunction);
                    d.blueLift.setTargetPosition(topJunction);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                    d.blueArm.setPosition(0.83);
                    d.blackArm.setPosition(0.83);
                    d.rotateServo.setPosition(1);
                })
                .splineToSplineHeading(new Pose2d(5, -8, Math.toRadians(45)), Math.toRadians(225))// TODO here
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    d.blueGripper.setPosition(0);
                    d.blackGripper.setPosition(0);
                })
                .waitSeconds(0.5)
                .build();

        TrajectorySequence leftSideParkLeft = d.trajectorySequenceBuilder(leftSide.end())
                .lineToSplineHeading(new Pose2d(-8, -7, Math.toRadians(270)))
                .lineToSplineHeading(new Pose2d(-56, -7, Math.toRadians(270)))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    d.blueArm.setPosition(lowerGripperPos);
                    d.blackArm.setPosition(lowerGripperPos);
                    d.rotateServo.setPosition(0);
                    d.blackLift.setTargetPosition(0);
                    d.blueLift.setTargetPosition(0);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                })
                .waitSeconds(0.5)
                .build(); // If left

        TrajectorySequence leftSideParkMiddle = d.trajectorySequenceBuilder(leftSide.end())
                .splineToSplineHeading(new Pose2d(-32, -7, Math.toRadians(270)), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    d.blueArm.setPosition(lowerGripperPos);
                    d.blackArm.setPosition(lowerGripperPos);
                    d.rotateServo.setPosition(0);
                    d.blackLift.setTargetPosition(0);
                    d.blueLift.setTargetPosition(0);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                })
                .waitSeconds(0.5)
                .build(); // If middle

        TrajectorySequence leftSideParkRight = d.trajectorySequenceBuilder(leftSide.end())
                .splineToSplineHeading(new Pose2d(-9, -7, Math.toRadians(270)), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    d.blueArm.setPosition(lowerGripperPos);
                    d.blackArm.setPosition(lowerGripperPos);
                    d.rotateServo.setPosition(0);
                    d.blackLift.setTargetPosition(0);
                    d.blueLift.setTargetPosition(0);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                })
                .waitSeconds(0.5)
                .build(); // If right

        return new TrajectorySequence[] {leftSide, leftSideParkLeft, leftSideParkMiddle, leftSideParkRight};
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
