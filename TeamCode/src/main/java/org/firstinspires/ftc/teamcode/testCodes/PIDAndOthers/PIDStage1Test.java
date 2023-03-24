package org.firstinspires.ftc.teamcode.testCodes.PIDAndOthers;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
@Config
@Disabled
public class PIDStage1Test extends LinearOpMode {
    double Kp = 0;
    double Ki = 0;
    double Kd = 0;

    int targetPosition = 0;

    DcMotorEx blueLift;
    DcMotorEx blackLift;

    PIDCoefficients coefficients = new PIDCoefficients(Kp, Ki, Kd);
    BasicPID controller = new BasicPID(coefficients);

    @Override
    public void runOpMode() throws InterruptedException {
        blackLift = hardwareMap.get(DcMotorEx.class, "blackLift");
        blueLift = hardwareMap.get(DcMotorEx.class, "blueLift");

        blueLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blackLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        blueLift.setDirection(DcMotorSimple.Direction.REVERSE);

        blueLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blackLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            blueLift.setPower(controller.calculate(targetPosition, blueLift.getCurrentPosition()));
            blackLift.setPower(controller.calculate(targetPosition, blackLift.getCurrentPosition()));
        }
    }
}
