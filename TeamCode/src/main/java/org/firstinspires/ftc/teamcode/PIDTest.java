package org.firstinspires.ftc.teamcode;

import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.LowPassFilter;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Disabled
public class PIDTest extends LinearOpMode {
    double Kp = 0;
    double Ki = 0;
    double Kd = 0;
    LowPassFilter lowPassFilter = new LowPassFilter(0.5);




    //PIDCoefficients coefficients = new PIDCoefficientsEx(Kp, Ki, Kd,)
    @Override
    public void runOpMode() throws InterruptedException {

    }
}
