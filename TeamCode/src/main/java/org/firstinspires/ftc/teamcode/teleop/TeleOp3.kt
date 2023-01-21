package org.firstinspires.ftc.teamcode.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.PIDController
import org.firstinspires.ftc.teamcode.subsystems.*
import org.firstinspires.ftc.teamcode.vision.ConeDetectionPipeline
import org.firstinspires.ftc.teamcode.vision.createWebcam
import org.openftc.easyopencv.OpenCvWebcam

@TeleOp
class TeleOp3 : LinearOpMode() {
    private lateinit var claw: Claw
    private lateinit var arm: Arm2
    private lateinit var drive: DriveExt
    private lateinit var gamepads: Pair<GamepadExt, GamepadExt>
    private val tm = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

    private lateinit var webcam: OpenCvWebcam
    private val pipeline = ConeDetectionPipeline(Alliance.RED)
    private val pickPID = PIDController(TeleOp4.pP, TeleOp4.pI, TeleOp4.pD)

    override fun runOpMode() {
        gamepads = GamepadExt(gamepad1) to GamepadExt(gamepad2)
        arm = Arm2(hardwareMap, tm)
        claw = Claw(hardwareMap)
        drive = DriveExt(hardwareMap)
        webcam = createWebcam(hardwareMap, RobotConfig.WEBCAM_1, pipeline = pipeline)

        val gp1 = gamepads.first
        val gp2 = gamepads.second

        EventLoop(::opModeIsActive).apply {
            onPressed(gp1::dpad_up) { arm.state = Arm.next(arm.state) }
            onPressed(gp1::dpad_down) { arm.state = Arm.prev(arm.state) }

            onPressed(gp1::a, gp2::a) { arm.state = Arm.GROUND }
            onPressed(gp1::x, gp2::x) { arm.state = Arm.MID }
            onPressed(gp1::y, gp2::y) { arm.state = Arm.HIGH }
            onPressed(gp1::b, gp2::b) { arm.state = Arm.BACKHIGH }

            onPressed(gp1::left_bumper, gp2::left_bumper) { claw.change() }

            /* aim at cone */
            var aiming = false
            onPressed(gp1::right_bumper, gp2::right_bumper) {
                aiming = (!aiming)
                if (aiming) claw.state = Claw.OPENED
            }
            runIf({ aiming }) {
                if (pipeline.detected) gp1.right_stick_x =
                    pickPID.calculate(pipeline.error).toFloat().also { println("aim $it") }
                if (claw.state == Claw.CLOSED) aiming = false
            }

            updates += listOf({ drive.update(gamepads) },
                { arm.update() },
                { tm.update(); Unit },
                { gamepads.sync() })

        }.also {
            waitForStart()
            it.run()
        }
    }
}
