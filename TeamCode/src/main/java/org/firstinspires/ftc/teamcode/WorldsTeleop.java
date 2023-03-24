package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.pastCodes.ConfigPos;
import org.firstinspires.ftc.teamcode.pastCodes.PoseStorage;

import java.util.ArrayList;

@Config
@TeleOp
public class WorldsTeleop extends LinearOpMode {
    SampleMecanumDrive d;

    int lowerLimit = 0;
    int upperLimit = 4000;

    double power;
    double liftPower;

    ArrayList<Boolean> booleanArray = new ArrayList<>();
    int booleanIncrementer = 0;
    boolean a2Pressed;
    boolean y2Pressed;
    boolean a1Pressed;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    WorldsConfig.gripper gripper = WorldsConfig.gripper.open;
    WorldsConfig.override over = WorldsConfig.override.no;
    WorldsConfig.rotation rotation = WorldsConfig.rotation.upright;
    WorldsConfig.arm arm = WorldsConfig.arm.forward;


    @Override
    public void runOpMode() throws InterruptedException {
        PhotonCore.enable();
        PhotonCore.experimental.setMaximumParallelCommands(6);
        PhotonCore.disable();

        d = new SampleMecanumDrive(hardwareMap);

        d.blueLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        d.blackLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        d.blueLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        d.blackLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        d.setPoseEstimate(PoseStorage.telePower);

        waitForStart();

        d.blueGripper.setPosition(0);
        d.blackGripper.setPosition(0);

        while(opModeIsActive() && !isStopRequested()) {
            drive();
            telemetry.addData("Blue lift", d.blueLift.getCurrentPosition());
            telemetry.addData("Black lift", d.blackLift.getCurrentPosition());
            telemetry.addData("Blue lift power", d.blueLift.getPower());
            telemetry.addData("Black lift power", d.blackLift.getPower());
            telemetry.update();

            liftPower = (gamepad2.right_trigger >= 0.5) ? 0.5 : 1;
            over = (gamepad2.left_trigger >= 0.5) ? WorldsConfig.override.yes : WorldsConfig.override.no;


            if (a2Pressed) {
                switch (gripper) {
                    case open:
                        d.blueGripper.setPosition(0.8);
                        d.blackGripper.setPosition(0.8);
                        gripper = WorldsConfig.gripper.closed;
                        break;
                    case closed:
                        d.blueGripper.setPosition(0);
                        d.blackGripper.setPosition(0);
                        gripper = WorldsConfig.gripper.open;
                        break;
                }

                /*if (gripper == WorldsConfig.gripper.open) {
                    d.blueGripper.setPosition(0.8);
                    d.blackGripper.setPosition(0.8);
                    gripper = WorldsConfig.gripper.closed;
                } else if (gripper == WorldsConfig.gripper.closed) {
                    d.blueGripper.setPosition(0);
                    d.blackGripper.setPosition(0);
                    gripper = WorldsConfig.gripper.open;
                }*/
            }

            if (y2Pressed) {
                d.blackLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                d.blueLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                d.blueLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                d.blueLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            if (a1Pressed) {
                d.setPoseEstimate(PoseStorage.telePower);
            }

            a2Pressed = ifPressed(gamepad2.a);
            y2Pressed = ifPressed(gamepad2.y);
            a1Pressed = ifPressed(gamepad1.a);
            booleanIncrementer = 0;
        }
    }

    private void drive() {
        // Read pose
        if (gamepad1.right_bumper) {
            power = 0.25;
        } else if (gamepad1.left_bumper) {
            power = 0.5;
        } else {
            power = 1;
        }

        Pose2d poseEstimate = d.getPoseEstimate();

        // Create a vector from the gamepad x/y inputs
        // Then, rotate that vector by the inverse of that heading
        Vector2d input = new Vector2d(
                -gamepad1.left_stick_y * power,
                -gamepad1.left_stick_x * power
        ).rotated(-poseEstimate.getHeading());

        // Pass in the rotated input + right stick value for rotation
        // Rotation is not part of the rotated input thus must be passed in separately
        d.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        -gamepad1.right_stick_x * power
                )
        );

        d.update();
    }

    //Function used to allow toggling
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
}