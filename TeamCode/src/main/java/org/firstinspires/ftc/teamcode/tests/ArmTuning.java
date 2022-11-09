package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp
public class ArmTuning extends OpMode {
    public static double kP = 0.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kCos = 0.0;

    public static double reference = 0.0;

    private final ElapsedTime timer = new ElapsedTime();
    private DcMotor motor;
    private DcMotor motor2;

    private final double TICKS_IN_DEG = 537.6 / 180;
    private PIDController control;

    @Override
    public void init() {
        motor = hardwareMap.dcMotor.get("lowlift");
        motor2 = hardwareMap.dcMotor.get("toplift");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        control = new PIDController(kP, kI, kD);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        control.setPID(kP, kI, kD);
        int armPos = motor.getCurrentPosition();
        double pid = control.calculate(armPos, reference);
        double ff = Math.cos(Math.toRadians(reference / TICKS_IN_DEG)) * kCos;

        double pow = pid + ff;
        motor.setPower(pid);
        motor2.setPower(pid);

        telemetry.addData("pos", armPos);
        telemetry.addData("ref", reference);
        telemetry.addData("pid", pid);
        telemetry.addData("ff", ff);
        telemetry.addData("pow", pow);
        telemetry.update();
    }
}
