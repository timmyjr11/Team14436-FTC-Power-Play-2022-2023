package org.firstinspires.ftc.teamcode.testCodes.movementAndFullRobot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pastCodes.ConfigPos;

import java.util.ArrayList;

@Config
@TeleOp
@Disabled
public class firstRobotTest extends LinearOpMode {
    DcMotorEx frontRight;
    DcMotorEx frontLeft;
    DcMotorEx backRight;
    DcMotorEx backLeft;
    DcMotorEx blueLift;
    DcMotorEx blackLift;
    Servo blueServo;
    Servo blackServo;

    int lowerLimit = 0;
    int upperLimit = 4000;

    double power;

    ArrayList<Boolean> booleanArray = new ArrayList<>();
    int booleanIncrementer = 0;
    boolean a1Pressed;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    ConfigPos.gripperPos gripper = ConfigPos.gripperPos.open;


    @Override
    public void runOpMode() throws InterruptedException {
        PhotonCore.enable();
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class,"backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        blackLift = hardwareMap.get(DcMotorEx.class, "blackLift");
        blueLift = hardwareMap.get(DcMotorEx.class, "blueLift");
        blackServo = hardwareMap.get(Servo.class, "blackServo");
        blueServo = hardwareMap.get(Servo.class, "blueServo");

        blueLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blackLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        blackLift.setDirection(DcMotorSimple.Direction.REVERSE);

        blueLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blackLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        blueLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blackLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode((DcMotor.RunMode.RUN_WITHOUT_ENCODER));
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        blueServo.setDirection(Servo.Direction.REVERSE);

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        blueServo.setPosition(0);
        blackServo.setPosition(0);


        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            setPower(gamepad1.left_stick_x,-gamepad1.left_stick_y, gamepad1.right_stick_x);

            if (gamepad1.dpad_up && blackLift.getCurrentPosition() < upperLimit && blueLift.getCurrentPosition() < upperLimit) {
                blackLift.setPower(0.5);
                blueLift.setPower(0.5);
            } else if (gamepad1.dpad_down && blueLift.getCurrentPosition() > lowerLimit && blackLift.getCurrentPosition() > lowerLimit) {
                blackLift.setPower(-0.5);
                blueLift.setPower(-0.5);
            } else {
                blueLift.setPower(0);
                blackLift.setPower(0);
            }

            if (a1Pressed) {
                if (gripper == ConfigPos.gripperPos.open) {
                    blueServo.setPosition(0.75);
                    blackServo.setPosition(0.75);
                    gripper = ConfigPos.gripperPos.closed;
                } else if (gripper == ConfigPos.gripperPos.closed) {
                    blueServo.setPosition(0);
                    blackServo.setPosition(0);
                    gripper = ConfigPos.gripperPos.open;
                }
            }
            telemetry.addData("Arming State: ", PhotonCore.CONTROL_HUB.getArmingState());
            telemetry.addData("Bulk Caching Mode", PhotonCore.CONTROL_HUB.getBulkCachingMode());
            telemetry.addData("Blinker pattern length", PhotonCore.CONTROL_HUB.getBlinkerPatternMaxLength());
            telemetry.update();
            a1Pressed = ifPressed(gamepad1.a);
            booleanIncrementer = 0;
        }
    }

    private void setPower(double y, double x, double rot) {
        double frontLeftPower = x + y + rot;
        double backLeftPower = x - y + rot;
        double frontRightPower = x - y - rot;
        double backRightPower = x + y - rot;

        if (gamepad1.right_bumper) {
            power = 0.5;
        } else if (gamepad1.left_bumper) {
            power = 0.25;
        } else {
            power = 1;
        }

        //Sets the power of the motors to the motors, which allows variable speed and movement in every direction
        frontLeft.setPower(frontLeftPower * power);
        frontRight.setPower(frontRightPower * power);
        backLeft.setPower(backLeftPower * power);
        backRight.setPower(backRightPower * power);
        telemetry.update();


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
