package org.firstinspires.ftc.teamcode.testCodes.movementAndFullRobot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pastCodes.ConfigPos;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.ArrayList;

@Config
@TeleOp
@Disabled
public class roadrunnerTeleopTest extends LinearOpMode {
    SampleMecanumDrive d;

    int lowerLimit = 0;
    int upperLimit = 4000;

    double power;
    double liftPower;

    ArrayList<Boolean> booleanArray = new ArrayList<>();
    int booleanIncrementer = 0;
    boolean a2Pressed;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    ConfigPos.gripperPos gripper = ConfigPos.gripperPos.open;


    @Override
    public void runOpMode() throws InterruptedException {
        PhotonCore.enable();
        d = new SampleMecanumDrive(hardwareMap);
        //d.setPoseEstimate(PoseStorage.telePowerRed);

        d.blueLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        d.blackLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        d.blueLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        d.blackLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        d.blueLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        d.blackLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        d.blueGripper.setPosition(0);
        d.blackGripper.setPosition(0);


        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            drive();
            if (gamepad2.right_trigger > 0.5) {
                liftPower = 0.5;
            } else {
                liftPower = 1;
            }

            if (gamepad2.dpad_up && d.blackLift.getCurrentPosition() < upperLimit && d.blueLift.getCurrentPosition() < upperLimit) {
                d.blackLift.setPower(1 * liftPower);
                d.blueLift.setPower(1 * liftPower);
            } else if (gamepad2.dpad_down && d.blueLift.getCurrentPosition() > lowerLimit && d.blackLift.getCurrentPosition() > lowerLimit) {
                d.blackLift.setPower(-1);
                d.blueLift.setPower(-1);
            } else {
                d.blueLift.setPower(0);
                d.blackLift.setPower(0);
            }

            if (a2Pressed) {
                if (gripper == ConfigPos.gripperPos.open) {
                    d.blueGripper.setPosition(0.75);
                    d.blackGripper.setPosition(0.75);
                    gripper = ConfigPos.gripperPos.closed;
                } else if (gripper == ConfigPos.gripperPos.closed) {
                    d.blueGripper.setPosition(0);
                    d.blackGripper.setPosition(0);
                    gripper = ConfigPos.gripperPos.open;
                }
            }
            telemetry.addData("Arming State: ", PhotonCore.CONTROL_HUB.getArmingState());
            telemetry.addData("Bulk Caching Mode", PhotonCore.CONTROL_HUB.getBulkCachingMode());
            telemetry.addData("Blinker pattern length", PhotonCore.CONTROL_HUB.getBlinkerPatternMaxLength());
            telemetry.update();
            a2Pressed = ifPressed(gamepad2.a);
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
