// A container for where WorldsAuto Resides
package org.firstinspires.ftc.teamcode.teachingCode;

// Other programs and code that WorldsAuto uses to operate
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.WorldsConfig; // Hehe, I wrote this lmao
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


// Annotations used to specify the type of program and allow for configurations via FTC Dashboard
@Config
@Autonomous
public class WorldsAutoComments extends LinearOpMode {
    /* An object based on the FtcDashboard class,
       used to monitor robot data and output */
    // TODO: Ask me to get the robot and the programming computer and robot ready
    private final FtcDashboard dash = FtcDashboard.getInstance();

    // An Enum used to determine colors, which is used for if statements
    static WorldsConfig.colors color = WorldsConfig.colors.None;

    /* SampleMecanumDrive is a class of RoadRunner we use it build our robot's autonomous
       You will have to learn how RoadRunner works and underlying ideas of how it works
       For right now, we declare an object named 'd' based on the SampleMecanumDrive class
     */
    //TODO: You can ask me if you want to understand SampleMecanumDrive, but it's a long one lol
    SampleMecanumDrive d;

    // We declare the two webcams that is used for OpenCV: A computer vision repository
    OpenCvWebcam blueCam;
    OpenCvWebcam blackCam;

    /* Integers used to determine the starting height the lift needs to be at and
       the height it needs to reach the top junction*/
    int liftLevel = 700;
    int topJunction = 1500;

    // Double used to have the servos for the arm in the starting position
    double lowerGripperPos = 0;

    // TODO: NOT USED IN FINAL PROGRAM
    /* Was used for adjusting the position needed for the robot to drop the cone onto the junction
       robot had drift due to bad odometry pods, so this was needed until I realized I needed to
       hardcode that stuff lmao */
    int leftForward = 0;
    int leftSideLevel = 0;
    int rightForward = 0;
    int rightSideLevel = 0;

    // Integer that allowed the driver to determine how many cones they wanted the robot to grab
    int coneCounter;

    // These enums were used so the drivers can tell the robot which side it is on
    // and what form of autonomous they wanted: Multi or single junction
    WorldsConfig.side side = WorldsConfig.side.tbd;
    WorldsConfig.type type = WorldsConfig.type.tbd;

    // An array that is used for controller toggling
    ArrayList<Boolean> booleanArray = new ArrayList<>();

    /// Arrays used to contain all of the RoadRunner paths
    // This does get explained as we go
    TrajectorySequence[] singleJunctionArray;
    TrajectorySequence[] multiJunctionArray;

    // Integer used to track the toggle inputs
    int booleanIncrementer = 0;

    // Booleans used for the d-pad up and down on the controller to determine if they are pressed
    boolean D1UpPressed;
    boolean D1DownPressed;

    // Boolean is used to determine if the robot is ready to run
    boolean ready = false;

    // Used to determine the size of the rectangle on the camera screen
    // used for data processing for OpenCV
    public static int rectX = 127;
    public static int rectY = 45; //95 If right
    public static int rectHeight = 100;
    public static int rectWidth = 75;

    /* doubles used as a part of a scalars to set the range of colors
       HSV stands for Hue, Saturation, and value
       lower is used for the lower range and
       upper is the upper range of colors
       goes from 0 - 255 for full HSV */
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

    /* Annotations to both stop warnings of deprecation (shown later) and override
       Override is a annotation used to tell the program that this method overrides its parent's
       method with the same name */
    // TODO: 'SuppressWarnings("deprecation")' is not needed, I just hate the look of deprecated methods
    @SuppressWarnings("deprecation")
    @Override
    // We FINALLY reach the runOpMode method.
    // TODO: Ask about "throws" and "InterruptedException" at your own risk
    //  as it will crate a whole new rant
    public void runOpMode() throws InterruptedException {

        // Connects the object "d" to the hardwareMap inside of the SampleMecanumDrive class,
        // which has all of our hardware configurations beforehand
        d = new SampleMecanumDrive(hardwareMap);

        // We set the lift motors to actively stay in place when there is no power
        // used to counter gravity
        d.blueLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        d.blackLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Resets the lift encoders, which tracks the position of the lift motors
        d.blueLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        d.blackLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Makes the motors run without its power being monitored by the encoder
        // allows for much more power output
        d.blueLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        d.blackLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Sets the robot into the starting position, finally
        d.blueArm.setPosition(lowerGripperPos);
        d.blackArm.setPosition(lowerGripperPos);
        d.rotateServo.setPosition(0);
        d.blueGripper.setPosition(0);
        d.blackGripper.setPosition(0);

        // Allows for the computer to track the movement and data of robot
        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        /* TODO: TIM NOTE. int cameraMonitorViewId = R.id.cameraMonitorViewId; is the better version
                "R" is the bridge between the code you write and
                the resources defined in XML and other files.*/
        // Collects the Ids of the cameras
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id",
                        hardwareMap.appContext.getPackageName());

        // Creates the Ids of the viewports for the two cameras
        // Viewports are what allows us to see what the camera sees
        int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
                .splitLayoutForMultipleViewports(cameraMonitorViewId,
                        2,
                        OpenCvCameraFactory.ViewportSplitMethod.HORIZONTALLY);

        // Just some statements that notify the drivers it is opening the cameras
        // Acts just like "print()" in Python or "System.out.Println()" in Java
        telemetry.addLine("Opening Cameras...");

        // Update is a method that notify the robot
        // to update the telemetry/data to the most recent statements
        // TODO: HAS to be used in Linear Op-mode
        telemetry.update();

        // Just like motors, this creates the connection between software and hardware
        blueCam = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), viewportContainerIds[0]);
        blackCam = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), viewportContainerIds[1]);

        // TODO: These are the statements that have deprecation: they are not Async
        //It's kinda in the statement... Opens the cameras.
        blueCam.openCameraDevice();
        blackCam.openCameraDevice();

        // Tells the drivers that the cameras have opened and then prompts the drivers for input
        // Asks the drivers which side on the field they are on
        telemetry.addLine("Cameras opened");
        telemetry.addLine("Press left on D-pad for left side, press right on D-pad for right side");
        telemetry.update();

        // TODO: Ask Tim what a while loop and a break statement does
        // Creates a while loop that continues to loop until there is a break
        while (true) {
            // Below is a nest of if statements that is used to configure the side the robot is on
            // TODO: This is super important as without it the auto cannot run properly

            // If driver presses left on D-Pad
            if (gamepad1.dpad_left) {
                // The robot's location is set the left side
                d.setPoseEstimate(WorldsConfig.leftAuto);

                // The side is then set to left using the enum WorldsConfig
                side = WorldsConfig.side.left;

                // The rectangle for the camera is changed for proper positioning
                rectY = 45;

                // Sets the OpenCV pipeline to the correct camera, pipeline at bottom of program
                blackCam.setPipeline(new ColorDetectionPipeline());

                // Allows the camera to collect video for data processing
                blackCam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);

                // Allows the camera video to be seen on the computer
                dash.startCameraStream(blackCam, 30);

                // At the end of this if statement
                // The break statement ejects the program out of the while loop
                break;
            } else if (gamepad1.dpad_right) {
                // Same as the if statement above, just for the right side
                d.setPoseEstimate(WorldsConfig.rightAuto);
                rectY = 95;
                side = WorldsConfig.side.right;
                blueCam.setPipeline(new ColorDetectionPipeline());
                blueCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                dash.startCameraStream(blueCam, 30);
                break;
            }
            // If the driver decides to end the program now, this if statement allows that
            if (isStopRequested()) return;
        }

        // Another while statement that loops until there is a break statement
        while (true) {
            // TODO: This is super important as without it the auto cannot run properly
            // Prompts the drivers, asking which version of auto they want: Single or Multi
            telemetry.addLine("What type of auto would you like to run?");
            telemetry.addLine("Press up on D-Pad for Single junction");
            telemetry.addLine("Press down on D-Pad for multi-junction");
            telemetry.update();

            // If the user presses up on D-pad, then the type Enum is set to single
            // Else if the user presses down on D-pad, then the type Enum is set to multi
            if (gamepad1.dpad_up) {
                type = WorldsConfig.type.singleJunction;

                // Allows the while statement to end
                break;
            } else if (gamepad1.dpad_down) {
                type = WorldsConfig.type.multiJunction;
                break;
            }
            // If the driver decides to end the program now, this if statement allows that
            if (isStopRequested()) return;
        }

        // If the type Enum is single
        if (type == WorldsConfig.type.singleJunction) {
            // While the driver has not pressed the "A" button on the controller
            while (!gamepad1.a) {
                // Telemetry used so the drivers knows how many cones they selected
                telemetry.addData("Number of cones: ", coneCounter);
                telemetry.addLine("Press up on D-pad to add a cone, press down on D-pad to remove a cone");
                telemetry.addLine("Press A to move on to the next step");

                /* If the driver presses up on D-pad and the coneCounter is less than 5,
                   then increment the coneCounter by 1
                   Else if the driver presses down on D-pad and the coneCounter is greater than 0
                   then increment the coneCounter by -1 */
                if (D1UpPressed && coneCounter < 5) {
                    coneCounter++;
                } else if (D1DownPressed && coneCounter > 0) {
                    coneCounter--;
                }

                // TODO: The statements below are based on the ifPressed method
                //  this is explained later on
                // Calls the ifPressed method using the D-pad as the argument
                D1UpPressed = ifPressed(gamepad1.dpad_up);
                D1DownPressed = ifPressed(gamepad1.dpad_down);

                // Explained in the ifPressed method
                booleanIncrementer = 0;

                telemetry.update();

                // If the driver decides to end the program now, this if statement allows that
                if (isStopRequested()) return;
            }

            // TODO: This is called a Ternary operator
            //  It's basically cool if statement that changes the value of a data type
            /* If the side Enum set to left, call the buildLeftSideSingle() method
               else call the buildRightSideSingle() method
               The results is then put into the singleJunctionArray */
            singleJunctionArray = (side == WorldsConfig.side.left) ? buildLeftSideSingle() : buildRightSideSingle();
        } else {
            // TODO: There wold be another Ternary operator that builds contains the multi right and left
            //  However, the right side multi was never completed, so it was never fully coded and used
            // Else call the buildLeftSideMulti() method and store the result in the multiJunctionArray
            multiJunctionArray = buildLeftSideMulti();
        }
        /* The robot is finally ready, which is shown to drivers
           The telemetry for it is located in the OpenCV Pipeline
           Why? Because the pipeline loops and that makes it perfect to show continuous telemetry */
        ready = true;

        // Robot holds initialization period until either the program ends
        // or the driver presses start
        while (!isStarted()) if (isStopRequested()) return;

        /* The robot stops the camera from processing any more video data
           This is because the pipeline loops, so if the robot moves the color could change
           This would causes madness for the auto */
        if (side == WorldsConfig.side.left) {
            blackCam.stopStreaming();
        } else {
            blueCam.stopStreaming();
        }

        /* TODO: This is where RoadRunner becomes necessary. Without it we would have to manually
            create a program that uses Bezier Curves and Inverse Kinematics to achieve the same result
            our robot's movement would have a fraction of the mobility without RoadRunner
            BTW: That's Calculus and University Physics level stuff :O */
        if (type == WorldsConfig.type.singleJunction) {
            // TODO: What this means is slightly explained below
            // Runs all of the movements created using the singleJunctionArray
            d.followTrajectorySequence(singleJunctionArray[0]);

            // Based on the number of the coneCounter, the robot does the statements that many
            // number of times
            if (coneCounter >= 1) d.followTrajectorySequence(singleJunctionArray[1]);
            liftLevel -= 140;
            if (coneCounter >= 2) d.followTrajectorySequence(singleJunctionArray[2]);
            liftLevel -= 140;
            if (coneCounter >= 3) d.followTrajectorySequence(singleJunctionArray[3]);
            liftLevel -= 140;
            if (coneCounter >= 4) d.followTrajectorySequence(singleJunctionArray[4]);
            liftLevel -= 140;
            if (coneCounter == 5) d.followTrajectorySequence(singleJunctionArray[5]);

            // TODO: Ask what a switch statement is
            // Based on the color of the cone, the robot parks in the correct spot
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
            // Runs all of the movements created using the multiJunctionArray
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
    // TODO: This will hurt your brain. Good luck explaining this future Tim
    // This method is what allows buttons to toggle.
    private boolean ifPressed(boolean button) {
        // The output of this method starts as false
        boolean output = false;

        // In the first loop, for each time the loop encounters an ifPressed method, the booleanArray
        // Increases by one, having that value in that index being false
        if (booleanArray.size() == booleanIncrementer) {
            booleanArray.add(false);
        }

        // buttonWas is the last boolean value that the button had
        boolean buttonWas = booleanArray.get(booleanIncrementer);

        // If the current button is not the same as the last button and the current button is true
        // the output will be true
        //noinspection PointlessBooleanExpression
        if (button != buttonWas && button == true) {
            output = true;
        }
        // Updates the value at that index to the current button
        booleanArray.set(booleanIncrementer, button);

        // Increments the booleanIncrementer by one
        booleanIncrementer = booleanIncrementer + 1;
        return output;
    }

    /* TODO: The next 814 lines of code is the building of the robot movements.
        For the sake of your and my sanity, I won't go into too much detail until we work with
        Roadrunner, but just know that:

        1. buildLeftSideSingle() and the related methods are a set of methods that create an Array
        of "Trajectory Sequences" that are inherently classes of Roadrunner.

        2. These methods can take A LOT of time to build, which is why we have that "ready" boolean,
        because if we start it too early the robot can crash. To circumvent this,
        we only ever "build" the Trajectory Sequences we need, which these methods help via "Encapsulation"

        3. An array is shown with "[]" and are indexed from zero. When we use:
        "d.followTrajectorySequence(singleJunctionArray[0]);"
        the singleJunctionArray[0] means that it will run the first Trajectory Sequence,
        with 0 being the first Trajectory Sequence, 1 being the second Trajectory Sequence, and so on.

        4. The methods are NOT statements, which means that although every variation is written,
        the only method that is "built" is the one we explicitly call.
     */
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

        TrajectorySequence leftSideCones3 = d.trajectorySequenceBuilder(leftSideCones2.end())
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


        TrajectorySequence leftSideCones4 = d.trajectorySequenceBuilder(leftSideCones3.end())
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

        TrajectorySequence leftSideCones5 = d.trajectorySequenceBuilder(leftSideCones4.end())
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
                .UNSTABLE_addTemporalMarkerOffset(0.1, () ->{
                    d.blueGripper.setPosition(1);
                    d.blackGripper.setPosition(1);
                })
                .splineToSplineHeading(new Pose2d(-34, -40, Math.toRadians(90)), Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    d.blackLift.setTargetPosition(1700);
                    d.blueLift.setTargetPosition(1700);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                })
                .splineToSplineHeading(new Pose2d(-38, -27, Math.toRadians(135)), Math.toRadians(90))
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
                .lineToSplineHeading(new Pose2d(-57.75, -8, Math.toRadians(180)))
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
                .splineToSplineHeading(new Pose2d(-31, -11, Math.toRadians(135)), Math.toRadians(315))
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
                .splineToSplineHeading(new Pose2d(-57.75, -7, Math.toRadians(180)), Math.toRadians(180))
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
                .splineToSplineHeading(new Pose2d(-7, -8, Math.toRadians(135)), Math.toRadians(315))
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    d.blueGripper.setPosition(0);
                    d.blackGripper.setPosition(0);
                })
                .waitSeconds(0.5)
                .build();

        TrajectorySequence leftSideParkLeft = d.trajectorySequenceBuilder(leftSide.end())
                .splineToSplineHeading(new Pose2d(-56, -7, Math.toRadians(270)), Math.toRadians(180))
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
                .splineToSplineHeading(new Pose2d(32.1, -4.75, Math.toRadians(315)), Math.toRadians(135))
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
                .splineToSplineHeading(new Pose2d(32.1, -6, Math.toRadians(315)), Math.toRadians(135))
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
                .splineToSplineHeading(new Pose2d(32.1, -6.75, Math.toRadians(315)), Math.toRadians(135))
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
                .splineToSplineHeading(new Pose2d(32.1, -7, Math.toRadians(315)), Math.toRadians(135))
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
                .splineToSplineHeading(new Pose2d(32.1, -7, Math.toRadians(315)), Math.toRadians(135))
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
                .splineToSplineHeading(new Pose2d(12, -11.5, Math.toRadians(270)), Math.toRadians(180))
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
// TODO: the code below was a test with switch cases rather than if statements. It did not work,
//  but I decided to keep it in a comment in case I ever come back to it.

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