package org.firstinspires.ftc.teamcode.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.PIDController
import org.firstinspires.ftc.teamcode.subsystems.Arm
import org.firstinspires.ftc.teamcode.subsystems.Arm.States.BACKMID
import org.firstinspires.ftc.teamcode.subsystems.Arm.States.GROUND
import org.firstinspires.ftc.teamcode.subsystems.Arm.States.LOW
import org.firstinspires.ftc.teamcode.subsystems.Arm.States.MID
import org.firstinspires.ftc.teamcode.subsystems.Claw
import org.firstinspires.ftc.teamcode.subsystems.Claw.States.CLOSED
import org.firstinspires.ftc.teamcode.subsystems.Claw.States.OPENED
import org.firstinspires.ftc.teamcode.subsystems.DriveExt
import org.firstinspires.ftc.teamcode.vision.ConeDetectionPipeline
import org.firstinspires.ftc.teamcode.vision.ConeDetectionPipeline.RED
import org.firstinspires.ftc.teamcode.vision.createWebcam
import org.openftc.easyopencv.OpenCvWebcam

@TeleOp
@com.acmerobotics.dashboard.config.Config
class TeleOp4 : LinearOpMode() {
    private lateinit var claw: Claw
    private lateinit var arm: Arm
    private lateinit var drive: DriveExt
    private lateinit var webcam: OpenCvWebcam
    private val pipeline = ConeDetectionPipeline(RED)
    private val pickPID = PIDController(pP, pI, pD)
    private val turnPID = PIDController(tP, tI, tD)

    private val tm = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

    override fun runOpMode() {
        arm = Arm(hardwareMap)
        claw = Claw(hardwareMap)
        drive = DriveExt(hardwareMap)
        webcam = createWebcam(hardwareMap, pipeline = pipeline)

        val gp1 = GamepadExt(gamepad1)
        val gp2 = GamepadExt(gamepad2)
        val gamepads = gp1 to gp2

        EventLoop(::opModeIsActive, tm).apply {
            updates += listOf(arm::update,
                { drive.update(gamepads) },
                gamepads::sync,
                { tm.update(); Unit })

            onPressed(gp1::left_bumper, gp2::left_bumper) { claw.change() }
            onPressed(gp1::dpad_up, gp2::dpad_up) { arm.state = Arm.next(arm.state) }
            onPressed(gp1::dpad_down, gp2::dpad_down) { arm.state = Arm.prev(arm.state) }
            onPressed(gp1::a, gp2::a) { arm.state = GROUND }
            onPressed(gp1::x, gp2::x) { arm.state = LOW }
            onPressed(gp1::y, gp2::y) { arm.state = MID }
            onPressed(gp1::b, gp2::b) { arm.state = BACKMID }

            /* aim at cone */
            var aiming = false
            onPressed(gp1::right_bumper, gp2::right_bumper) {
                aiming = (!aiming)
                if (aiming) claw.state = OPENED
            }
            runIf({ aiming }) {
                if (pipeline.detected) gp1.right_stick_x =
                    pickPID.calculate(pipeline.error).toFloat().also { println("aim $it") }
                if (claw.state == CLOSED) aiming = false
            }

            fun wrapAngle(ang: Double): Double {
                if (ang > 360) return ang - 360
                if (ang > 180) return -360 + ang
                return ang
            }

            val ndeg = 361.0

            /* turn 180 left */
            var leftAngle = ndeg

            onPressed(gp1::dpad_left) {
                leftAngle = if (leftAngle == ndeg) wrapAngle(drive.poseEstimate.heading + 180)
                else ndeg
                println("left target : $leftAngle")
            }

            runIf({ leftAngle != ndeg && gp1.right_stick_x == 0F }) {
                gp1.right_stick_x =
                    turnPID.calculate(leftAngle, wrapAngle(drive.poseEstimate.heading)).toFloat()
                        .also { println("left : ${wrapAngle(drive.poseEstimate.heading)} $it") }
                if (wrapAngle(drive.poseEstimate.heading) in (leftAngle - 2)..(leftAngle + 2)) leftAngle =
                    ndeg
            }

            /* turn 180 right */
            var rightAngle = ndeg

            onPressed(gp1::dpad_right) {
                rightAngle = if (rightAngle == ndeg) wrapAngle(drive.poseEstimate.heading - 180)
                else ndeg
                println("right target : $rightAngle")
            }

            runIf({ rightAngle != ndeg && gp1.right_stick_x == 0F }) {
                gp1.left_stick_x =
                    turnPID.calculate(rightAngle, wrapAngle(drive.poseEstimate.heading)).toFloat()
                        .also { println("right : ${wrapAngle(drive.poseEstimate.heading)} - $it") }
                if (wrapAngle(drive.poseEstimate.heading) in (rightAngle - 2)..(rightAngle + 2)) rightAngle =
                    ndeg
            }
        }.also {
            waitForStart()
            it.run()
        }
    }

    companion object {
        @JvmField var pP = 0.002
        @JvmField var pI = 0.0
        @JvmField var pD = 0.0

        @JvmField var tP = 0.01
        @JvmField var tI = 0.0
        @JvmField var tD = 0.0
    }
}
