package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Config;
import org.firstinspires.ftc.teamcode.robot.PIDController;

import kotlin.Pair;
import kotlin.Triple;

@TeleOp
@com.acmerobotics.dashboard.config.Config
public class ArmTuning extends OpMode {
    public static double kP = 0.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kCos = 0.0;
    public static int setpoint = 0;

    public static double TICKS_IN_DEGREES = 0.0;
    public static double diff = 0.0;

    private DcMotorEx low, top;
    private final PIDController control = new PIDController(new PIDController.Coefficients(0, 0, 0));

    private final MultipleTelemetry tm = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    @Override
    public void init() {
        low = (DcMotorEx) hardwareMap.get(Config.LOW_LIFT.getS());
        top = (DcMotorEx) hardwareMap.get(Config.TOP_LIFT.getS());

        low.setDirection(DcMotorSimple.Direction.REVERSE);
        top.setDirection(DcMotorSimple.Direction.REVERSE);
        low.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        top.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        low.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        top.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    @Override
    public void loop() {
        control.setCoefs(new PIDController.Coefficients(kP, kI, kD));
        control.setSetpoint(setpoint);
        Pair<Double, Triple<Double, Double, Double>> res = control.calculate(low.getCurrentPosition());
        Double pid = res.component1();
        Double p = res.component2().component1();
        Double i = res.component2().component2();
        Double d = res.component2().component3();

        Double ff = kCos * Math.cos(Math.toRadians((setpoint - diff) / TICKS_IN_DEGREES));

        low.setPower(pid + ff);
        top.setPower(pid + ff);

        tm.addData("p", p);
        tm.addData("i", i);
        tm.addData("d", d);
        tm.addData("pid", pid);
        tm.addData("ff", ff);
        tm.addData("pidf", pid + ff);

        tm.addData("currentPos", low.getCurrentPosition());
        tm.addData("setpoint", setpoint);

        tm.update();
    }
}
