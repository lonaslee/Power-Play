package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.robot.Config;

@TeleOp
@com.acmerobotics.dashboard.config.Config
public class ArmTuning2 extends OpMode {
    public static double kP = 0.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kCos = 0.0;

    public static double aP = 0.0;
    public static double aI = 0.0;
    public static double aD = 0.0;
    public static double aCos = 0.0;

    private static double lastsetpoint = 0.0;
    public static double setpoint = 0.0;

    private DcMotorEx low, top;
    private final PIDFController upControl = new PIDFController(kP, kI, kD, kCos);
    private final PIDFController downControl = new PIDFController(aP, aI, aD, aCos);
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

    private boolean goingDown = false;

    @Override
    public void loop() {
        upControl.setPIDF(kP, kI, kD, kCos);
        upControl.setSetPoint(setpoint);
        downControl.setPIDF(aP, aI, aD, aCos);
        downControl.setSetPoint(setpoint);

        double upPIDF = upControl.calculate(low.getCurrentPosition());
        double downPIDF = downControl.calculate(low.getCurrentPosition());

        double power;
        if (goingDown) {
            telemetry.addLine("USING DOWN");
            power = downPIDF;
        } else {
            telemetry.addLine("uSING UP");
            power = upPIDF;
        }

        low.setPower(power);
        top.setPower(power);

        tm.addData("_currentPos", low.getCurrentPosition());
        tm.addData("_setpoint", setpoint);

        tm.update();
        if (lastsetpoint != setpoint) {
            if (lastsetpoint > setpoint) goingDown = true;
            else goingDown = false;
            lastsetpoint = setpoint;
        }
    }
}
