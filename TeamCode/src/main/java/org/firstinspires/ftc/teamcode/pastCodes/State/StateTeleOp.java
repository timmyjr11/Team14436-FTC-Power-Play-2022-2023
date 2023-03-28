package org.firstinspires.ftc.teamcode.pastCodes.State;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pastCodes.ConfigPos;
import org.firstinspires.ftc.teamcode.pastCodes.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.ArrayList;

@Config
@TeleOp
public class StateTeleOp extends LinearOpMode {
    SampleMecanumDrive d;

    int lowerLimit = 0;
    int upperLimit = 1135;
    int slowLimit = 367;

    double power;
    double liftPower;

    ArrayList<Boolean> booleanArray = new ArrayList<>();
    int booleanIncrementer = 0;
    boolean a2Pressed;
    boolean y2Pressed;
    boolean a1Pressed;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    ConfigPos.gripperPos gripper = ConfigPos.gripperPos.open;
    ConfigPos.override over = ConfigPos.override.no;


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

            if (gamepad2.right_trigger > 0.5) {
                liftPower = 0.5;
            } else {
                liftPower = 1;
            }

            if (gamepad2.left_trigger > 0.5) {
                over = ConfigPos.override.yes;
            } else {
                over = ConfigPos.override.no;
            }

            if ((gamepad2.dpad_up && over == ConfigPos.override.yes) || (gamepad2.dpad_up && d.blackLift.getCurrentPosition() < upperLimit && d.blueLift.getCurrentPosition() < upperLimit)) {
                d.blackLift.setPower(1 * liftPower);
                d.blueLift.setPower(1 * liftPower);
            } else if ((gamepad2.dpad_down && over == ConfigPos.override.yes) || (gamepad2.dpad_down && d.blueLift.getCurrentPosition() > lowerLimit && d.blackLift.getCurrentPosition() > lowerLimit)) {

                if (d.blueLift.getCurrentPosition() <= slowLimit && d.blackLift.getCurrentPosition() <= slowLimit) {
                    liftPower = 0.35;
                } else if (gamepad2.right_trigger >= 0.5) {
                    liftPower = 0.5;
                } else {
                    liftPower = 1;
                }

                d.blackLift.setPower(-1 * liftPower);
                d.blueLift.setPower(-1 * liftPower);
            } else {
                d.blueLift.setPower(0.15);
                d.blackLift.setPower(0.15);
            }

            if (a2Pressed) {
                if (gripper == ConfigPos.gripperPos.open) {
                    d.blueGripper.setPosition(0.8);
                    d.blackGripper.setPosition(0.8);
                    gripper = ConfigPos.gripperPos.closed;
                } else if (gripper == ConfigPos.gripperPos.closed) {
                    d.blueGripper.setPosition(0);
                    d.blackGripper.setPosition(0);
                    gripper = ConfigPos.gripperPos.open;
                }
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