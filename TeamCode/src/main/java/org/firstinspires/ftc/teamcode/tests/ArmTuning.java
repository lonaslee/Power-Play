package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp
public class ArmTuning extends LinearOpMode {
    public static double kP = 0.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kCos = 0.0;

    public static double reference = 0.0;

    private final ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        DcMotor motor = hardwareMap.dcMotor.get("lowlift");
        DcMotor motor2 = hardwareMap.dcMotor.get("toplift");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        PIDController control = new PIDController(kP, kI, kD);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            control.setPID(kP, kI, kD);
            int armPos = motor.getCurrentPosition();
            double pid = control.calculate(armPos, reference);
            double TICKS_IN_DEG = 537.6 / 180;
            double ff = Math.cos(Math.toRadians(reference / TICKS_IN_DEG)) * kCos;

            double pow = pid + ff;
            motor.setPower(pow);
            motor2.setPower(pow);

            telemetry.addData("pos", armPos);
            telemetry.addData("ref", reference);
            telemetry.addData("pid", pid);
            telemetry.addData("ff", ff);
            telemetry.addData("pow", pow);
            telemetry.update();
            System.out.println("i");

        }
    }
}
