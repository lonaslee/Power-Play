package org.firstinspires.ftc.teamcode.teleop;

import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.toRadians;
import static kotlin.system.TimingKt.measureNanoTime;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PIDController;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.AnglePresets;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.RobotConfig;

import java.util.List;

import kotlin.Triple;

@TeleOp(group = "test")
@com.acmerobotics.dashboard.config.Config
public class Test extends LinearOpMode {
    private SampleMecanumDrive drive = null;
    private Servo claw = null;

    private List<DcMotor> motors = null;

    private int armTarget = AnglePresets.aGROUND;
    private int lastArmTarget = AnglePresets.aGROUND;

    private MotionState motionState = new MotionState(AnglePresets.aGROUND, 0, 0);
    private MotionProfile motionProfile =
            MotionProfileGenerator.generateSimpleMotionProfile(motionState, motionState, 0, 0);

    private final ElapsedTime timer = new ElapsedTime();
    private final PIDController controller = new PIDController(aP, bI, cD);

    private final Telemetry tm =
            new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    private final Gamepad lastGamepad1 = new Gamepad();

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        claw = hardwareMap.servo.get(RobotConfig.CLAW.s);
        motors = List.of(hardwareMap.dcMotor.get(RobotConfig.TOP_LIFT.s),
                hardwareMap.dcMotor.get(RobotConfig.LOW_LIFT.s)
        );
        motors.forEach(it -> {
            it.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            it.setDirection(DcMotorSimple.Direction.REVERSE);
            it.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            it.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        });

        waitForStart();
        while (opModeIsActive()) tm.addData("hz", measureNanoTime(() -> {
            var x = gamepad1.left_stick_x;
            var y = gamepad1.left_stick_y;
            var heading = drive.getRawExternalHeading();

            drive.setWeightedDrivePower(new Pose2d(x * cos(heading) - y * sin(heading),
                    x * sin(heading) - y * cos(heading),
                    gamepad1.right_stick_x
            ));

            if (gamepad1.left_bumper && !lastGamepad1.left_bumper) {
                if (claw.getPosition() == Claw.CLOSED) claw.setPosition(Claw.OPENED);
                else claw.setPosition(Claw.CLOSED);
            }

            if (gamepad1.a && !lastGamepad1.a) armTarget = AnglePresets.aGROUND;
            else if (gamepad1.x && !lastGamepad1.x) armTarget = AnglePresets.cMID;
            else if (gamepad1.y && !lastGamepad1.y) armTarget = AnglePresets.dHIGH;
            else if (gamepad1.b && !lastGamepad1.b) armTarget = AnglePresets.eBACKHIGH;
            else if (gamepad1.dpad_up && !lastGamepad1.dpad_up)
                armTarget = AnglePresets.INSTANCE.next(armTarget);
            else if (gamepad1.dpad_down && !lastGamepad1.dpad_down)
                armTarget = AnglePresets.INSTANCE.prev(armTarget);

            if (lastArmTarget != armTarget) {
                var totalDistance = abs(lastArmTarget - armTarget);
                var startPosition = motionState.getX();

                motionProfile = MotionProfileGenerator.generateMotionProfile(motionState,
                        new MotionState(armTarget, 0, 0),
                        (s) -> gMAX_VEL,
                        (s) -> {
                            double percent = abs(startPosition - s) / totalDistance;
                            if (percent < 0.8) return hMAX_ACCEL;
                            return hMAX_ACCEL * (1 - percent) / 0.2;
                        }
                );
                timer.reset();
            }

            var armPos = motors.get(0).getCurrentPosition();
            var angle = armPos / eTICKS_IN_DEGREES + fANG_DISP;

            motionState = motionProfile.get(timer.time());

            var pid = controller.calculate(motionState.getX(), angle);
            var ff = dCos * cos(toRadians(angle));

            var pow = pid + ff;
            motors.forEach(it -> it.setPower(pow));

            tm.addData("_angle", angle);
            tm.addData("_target", motionState.getX());
            tm.addData("ticks", armPos);

            drive.update();
            tm.update();

            controller.setConstants(new Triple<>(aP, bI, cD));
            lastGamepad1.copy(gamepad1);
            lastArmTarget = armTarget;
            return null;
        }));
    }

    public static double aP = 0.001;
    public static double bI = 0.0;
    public static double cD = 0.0;
    public static double dCos = 0.015;
    public static double eTICKS_IN_DEGREES = 90 / 220.0;
    public static double fANG_DISP = 70;
    public static double gMAX_VEL = 10;
    public static double hMAX_ACCEL = 10;
}
