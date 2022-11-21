package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.robot.FFController;
import org.firstinspires.ftc.teamcode.robot.PIDController;

import java.util.Arrays;
import java.util.List;

@TeleOp(name = "A_PidfTest")
@com.acmerobotics.dashboard.config.Config
public class PidfTest extends OpMode {
    private final PIDController pidControl = new PIDController(new PIDController.Coefficients(0, 0, 0));
    private final FFController ffControl = new FFController(new FFController.Coefficients(0, 0));

    public static double kP = 0.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kV = 0.0;
    public static double kA = 0.0;

    public static double MAX_VELO = 0.0;
    public static double MAX_ACCEL = 0.0;


    public static double target = 0.0;

    private DcMotorEx low, top;
    private List<DcMotorEx> motors;

    @Override
    public void init() {
        low = (DcMotorEx) hardwareMap.get("lowlift");
        top = (DcMotorEx) hardwareMap.get("toplift");
        motors = Arrays.asList(low, top);
        motors.forEach(it -> {
            it.setDirection(DcMotorSimple.Direction.REVERSE);
            it.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            it.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        });
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }
    @Override
    public void loop() {
        pidControl.setSetpoint((int) target);
        ffControl.setSetpoint((int) target);
        pidControl.setCoefs(new PIDController.Coefficients(kP, kI, kD));
        ffControl.setCoefs(new FFController.Coefficients(kA, kV));
        FFController.Companion.setMAX_VELO(MAX_VELO);
        FFController.Companion.setMAX_ACCEL(MAX_ACCEL);

        double pid = pidControl.calculate(low.getCurrentPosition()).component1();
        double ff = ffControl.calculate(low.getCurrentPosition());

        motors.forEach(it -> it.setPower(pid + ff));

        telemetry.addData("pid", pid);
        telemetry.addData("ff", ff);
        telemetry.addData("target", target);
        telemetry.addData("current", low.getCurrentPosition());
        telemetry.update();
    }
}
