package org.firstinspires.ftc.teamcode.pastCodes.Cactus;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pastCodes.ConfigPos;
import org.firstinspires.ftc.teamcode.pastCodes.PoseStorage;
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
@Disabled
public class CrackedAutoBlue extends LinearOpMode {

    private final FtcDashboard dash = FtcDashboard.getInstance();

    static ConfigPos.colors color = ConfigPos.colors.None;
    static ConfigPos.colors previousColor = ConfigPos.colors.None;


    SampleMecanumDrive d;

    OpenCvWebcam blueCam;
    OpenCvWebcam blackCam;

    int liftLevel = 650;

    int coneCounter;

    ConfigPos.side side = ConfigPos.side.tbd;

    ArrayList<Boolean> booleanArray = new ArrayList<>();
    int booleanIncrementer = 0;
    boolean D1UpPressed;
    boolean D1DownPressed;

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
                d.setPoseEstimate(PoseStorage.leftAuto);
                side = ConfigPos.side.left;
                rectY = 45;
                blackCam.setPipeline(new ColorDetectionPipelineV2());
                blackCam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
                dash.startCameraStream(blackCam, 30);
                break;
            } else if (gamepad1.dpad_right) {
                d.setPoseEstimate(PoseStorage.rightAuto);
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
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> {
                    d.blueGripper.setPosition(0.8);
                    d.blackGripper.setPosition(0.8);
                })
                .splineToConstantHeading(new Vector2d(-35, -59), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-25, -59), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-14, -55), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-14, -30), Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    d.blackLift.setTargetPosition(4000);
                    d.blueLift.setTargetPosition(4000);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(0.8);
                    d.blueLift.setPower(0.8);
                })
                .turn(Math.toRadians(-40))
                .forward(6)
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    d.blueGripper.setPosition(0);
                    d.blackGripper.setPosition(0);
                })
                .lineToLinearHeading(new Pose2d(-14, -30, Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    d.blackLift.setTargetPosition(0);
                    d.blueLift.setTargetPosition(0);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(0.8);
                    d.blueLift.setPower(0.8);
                })
                .lineToConstantHeading(new Vector2d(-14, -8))
                .turn(Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(-38, -8))
                .build();

        TrajectorySequence leftGrabCones = d.trajectorySequenceBuilder(leftSide.end())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    d.blackLift.setTargetPosition(liftLevel);
                    d.blueLift.setTargetPosition(liftLevel);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(0.8);
                    d.blueLift.setPower(0.8);
                })
                .lineToConstantHeading(new Vector2d(-65, -8))
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> {
                    d.blackGripper.setPosition(0.8);
                    d.blueGripper.setPosition(0.8);
                })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    d.blackLift.setTargetPosition(4000);
                    d.blueLift.setTargetPosition(4000);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(0.8);
                    d.blueLift.setPower(0.8);
                })
                .lineToConstantHeading(new Vector2d(-35, -8))
                .turn(Math.toRadians(-115))
                .forward(5)
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    d.blueGripper.setPosition(0);
                    d.blackGripper.setPosition(0);
                })
                .lineToSplineHeading(new Pose2d(-40, -8, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(Math.toRadians(250)))
                .UNSTABLE_addTemporalMarkerOffset(-0.75, () -> {
                    d.blackLift.setTargetPosition(liftLevel);
                    d.blueLift.setTargetPosition(liftLevel);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(0.8);
                    d.blueLift.setPower(0.8);
                })
                .build();

        TrajectorySequence LeftParking = d.trajectorySequenceBuilder(leftGrabCones.end())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    d.blackLift.setTargetPosition(0);
                    d.blueLift.setTargetPosition(0);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(0.8);
                    d.blueLift.setPower(0.8);
                })
                .turn(Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(-40, -30, Math.toRadians(270)))
                .build();

        TrajectorySequence LeftParkingPosOne = d.trajectorySequenceBuilder(LeftParking.end())
                .lineToConstantHeading(new Vector2d(-68, -30))
                .build();
        TrajectorySequence LeftParkingPosTwo = d.trajectorySequenceBuilder(LeftParking.end())
                .lineToConstantHeading(new Vector2d(-11, -30))
                .build();





        TrajectorySequence rightSide = d.trajectorySequenceBuilder(d.getPoseEstimate())
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> {
                    d.blueGripper.setPosition(0.8);
                    d.blackGripper.setPosition(0.8);
                })
                .splineToConstantHeading(new Vector2d(35, -59), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(25, -59), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(15, -55), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(15, -30), Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    d.blackLift.setTargetPosition(4000);
                    d.blueLift.setTargetPosition(4000);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(0.8);
                    d.blueLift.setPower(0.8);
                })
                .turn(Math.toRadians(45))
                .forward(6.5)
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    d.blueGripper.setPosition(0);
                    d.blackGripper.setPosition(0);
                })
                .lineToLinearHeading(new Pose2d(15, -30, Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    d.blackLift.setTargetPosition(0);
                    d.blueLift.setTargetPosition(0);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(0.8);
                    d.blueLift.setPower(0.8);
                })
                .lineToConstantHeading(new Vector2d(15, -8))
                .turn(Math.toRadians(-90))
                .lineToConstantHeading(new Vector2d(38, -8))
                .build();

        TrajectorySequence rightGrabCones = d.trajectorySequenceBuilder(rightSide.end())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    d.blackLift.setTargetPosition(liftLevel);
                    d.blueLift.setTargetPosition(liftLevel);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(0.8);
                    d.blueLift.setPower(0.8);
                })
                .lineToConstantHeading(new Vector2d(65, -8))
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> {
                    d.blackGripper.setPosition(0.8);
                    d.blueGripper.setPosition(0.8);
                })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    d.blackLift.setTargetPosition(4000);
                    d.blueLift.setTargetPosition(4000);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(0.8);
                    d.blueLift.setPower(0.8);
                })
                .lineToConstantHeading(new Vector2d(35, -8))
                .turn(Math.toRadians(120))
                .forward(5)
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    d.blueGripper.setPosition(0);
                    d.blackGripper.setPosition(0);
                })
                .back(5)
                .lineToSplineHeading(new Pose2d(38, -8, Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(Math.toRadians(250)))
                .UNSTABLE_addTemporalMarkerOffset(-0.75, () -> {
                    d.blackLift.setTargetPosition(liftLevel);
                    d.blueLift.setTargetPosition(liftLevel);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(0.8);
                    d.blueLift.setPower(0.8);
                })
                .build();

        TrajectorySequence RightParking = d.trajectorySequenceBuilder(rightGrabCones.end())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    d.blackLift.setTargetPosition(0);
                    d.blueLift.setTargetPosition(0);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(0.8);
                    d.blueLift.setPower(0.8);
                })
                .turn(Math.toRadians(-90))
                .lineToLinearHeading(new Pose2d(38, -30, Math.toRadians(270)))
                .build();

        TrajectorySequence RightParkingPosOne = d.trajectorySequenceBuilder(RightParking.end())
                .lineToConstantHeading(new Vector2d(11, -30))
                .build();
        TrajectorySequence RightParkingPosTwo = d.trajectorySequenceBuilder(RightParking.end())
                .lineToConstantHeading(new Vector2d(68, -30))
                .build();

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


        if (side == ConfigPos.side.left) {
            blackCam.stopStreaming();
        } else if (side == ConfigPos.side.right) {
            blueCam.stopStreaming();
        }

        if (side == ConfigPos.side.left) {
            d.followTrajectorySequence(leftSide);
            if (coneCounter > 0) {
                for (int i = 0; i < coneCounter; i++) {
                    d.followTrajectorySequence(leftGrabCones);
                    liftLevel -= 200;
                }
            }
            d.followTrajectorySequence(LeftParking);
            if (color == ConfigPos.colors.green) {
                d.followTrajectorySequence(LeftParkingPosOne);
            } else if (color == ConfigPos.colors.purple) {
                d.followTrajectorySequence(LeftParkingPosTwo);
            }
        } else if (side == ConfigPos.side.right) {
            d.followTrajectorySequence(rightSide);
            if (coneCounter > 0) {
                for (int i = 0; i < coneCounter; i++) {
                    d.followTrajectorySequence(rightGrabCones);
                    liftLevel -= 200;
                }
            }
            d.followTrajectorySequence(RightParking);
            if (color == ConfigPos.colors.green) {
                d.followTrajectorySequence(RightParkingPosOne);
            } else if (color == ConfigPos.colors.purple) {
                d.followTrajectorySequence(RightParkingPosTwo);
            }
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
            telemetry.update();
            return input;
        }
    }
}