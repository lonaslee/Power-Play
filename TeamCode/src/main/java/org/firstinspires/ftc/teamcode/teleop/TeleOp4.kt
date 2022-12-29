package org.firstinspires.ftc.teamcode.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.controller.PIDController
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.robot.*
import org.firstinspires.ftc.teamcode.robot.Arm.States.BACKMID
import org.firstinspires.ftc.teamcode.robot.Arm.States.GROUND
import org.firstinspires.ftc.teamcode.robot.Arm.States.LOW
import org.firstinspires.ftc.teamcode.robot.Arm.States.MID
import org.firstinspires.ftc.teamcode.robot.Claw.States.CLOSED
import org.firstinspires.ftc.teamcode.vision.ConeDetectionPipeline
import org.firstinspires.ftc.teamcode.vision.ConeDetectionPipeline.RED
import org.firstinspires.ftc.teamcode.vision.createWebcam
import org.openftc.easyopencv.OpenCvWebcam

@TeleOp
class TeleOp4 : LinearOpMode() {
    private lateinit var claw: Claw
    private lateinit var arm: Arm
    private lateinit var drive: DriveExt
    private lateinit var webcam: OpenCvWebcam
    private val pipeline = ConeDetectionPipeline(RED)
    private val pickPID = PIDController(pP, pI, pD).apply { setPoint = 0.0 }

    private val tm = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

    override fun runOpMode() {
        arm = Arm(hardwareMap)
        claw = Claw(hardwareMap)
        drive = DriveExt(hardwareMap)
        webcam = createWebcam(hardwareMap, pipeline = pipeline)

        val gp1 = GamepadExt(gamepad1)
        val gp2 = GamepadExt(gamepad2)
        val gamepads = gp1 to gp2

        EventLoop(::opModeIsActive).apply {
            updates += mutableListOf(arm::update, { drive.update(gamepads) }, {
                if (claw.state == Claw.States.OPENED && arm.state > MID) claw.halfOpen()
                else if (claw.state == Claw.States.HALF_OPENED && arm.state <= MID) claw.open()
            }, gamepads::sync)

            onAnyPressed(gp1::left_bumper, gp2::left_bumper) { claw.change() }
            onAnyPressed(gp1::dpad_up, gp2::dpad_up) { arm.state = Arm.States.next(arm.state).toInt() }
            onAnyPressed(gp1::dpad_down, gp2::dpad_down) { arm.state = Arm.States.prev(arm.state).toInt() }
            onAnyPressed(gp1::a, gp2::a) { arm.state = GROUND }
            onAnyPressed(gp1::x, gp2::x) { arm.state = LOW }
            onAnyPressed(gp1::y, gp2::y) { arm.state = MID }
            onAnyPressed(gp1::b, gp2::b) { arm.state = BACKMID }

            object : EventLoop.Callback {
                private var enabled = false
                override fun invoke() {
                    enabled = !enabled
                }

                fun update(gamepads: Gamepads) {
                    if (pipeline.detected) {
                        pickPID.setPID(TeleOp3.pP, TeleOp3.pI, TeleOp3.pD)
                        gamepads.first.right_stick_x = -pickPID.calculate(pipeline.error).toFloat()
                    }
                    if (claw.state == CLOSED) enabled = false
                }
            }.also {
                updates += { it.update(gamepads) }
                onAnyPressed(gp1::right_bumper, gp2::right_bumper) {
                    it()
                }
            }
        }.run()
    }

    companion object {
        @JvmField var pP = 0.002
        @JvmField var pI = 0.0
        @JvmField var pD = 0.0
    }
}
