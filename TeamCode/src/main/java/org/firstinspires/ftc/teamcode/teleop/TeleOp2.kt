package org.firstinspires.ftc.teamcode.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.PIDController
import org.firstinspires.ftc.teamcode.subsystems.*
import org.firstinspires.ftc.teamcode.vision.*
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvWebcam

@TeleOp(group = "test")
@com.acmerobotics.dashboard.config.Config
class TeleOp2 : LinearOpMode() {
    private lateinit var claw: Claw
    private lateinit var arm: Arm3
    private lateinit var drive: DriveExt
    private lateinit var odoRetract: OdoRetract
    private lateinit var gamepads: Pair<GamepadExt, GamepadExt>
    private val tm = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

    private lateinit var frontWebcam: OpenCvWebcam
    private lateinit var backWebcam: OpenCvWebcam
    private val coneDetector = ConeDetectionPipeline(ConeDetectionPipeline.ConeColor.RED, tm)
    private val poleDetector = PoleDetectionPipeline(tm)
    private val conePID = PIDController(pP, pI, pD)
    private val polePID = PIDController(jP, jI, jD)
    override fun runOpMode() {
        gamepads = GamepadExt(gamepad1) to GamepadExt(gamepad2)
        arm = Arm3(hardwareMap, tm)
        claw = Claw(hardwareMap)
        drive = DriveExt(hardwareMap)
        odoRetract = OdoRetract(hardwareMap, tm)

        backWebcam = createWebcam(
            hardwareMap, RobotConfig.WEBCAM_2, pipeline = poleDetector,
        )
        frontWebcam = createWebcam(
            hardwareMap, RobotConfig.WEBCAM_1, pipeline = coneDetector
        )

        val gp1 = gamepads.first
        val gp2 = gamepads.second

        EventLoop(::opModeIsActive, tm).apply {
            onPressed(gp1::dpad_up) { arm.state = Arm.next(arm.state) }
            onPressed(gp1::dpad_down) { arm.state = Arm.prev(arm.state) }

            onPressed(gp1::a, gp2::a) { arm.state = Arm.GROUND }
            onPressed(gp1::x, gp2::x) { arm.state = Arm.MID }
            onPressed(gp1::y, gp2::y) { arm.state = Arm.HIGH }
            onPressed(gp1::b, gp2::b) { arm.state = Arm.BACKHIGH }

            onPressed(gp1::left_bumper, gp2::left_bumper) { claw.change() }

            onPressed(gp1::dpad_right) {
                odoRetract.state =
                    if (odoRetract.state == OdoRetract.DOWN) OdoRetract.RETRACTED else OdoRetract.DOWN
            }

            /* sprint mode */
            onMoved(gp1::left_trigger) { drive.state = DriveExt.SPRINTING }
            onReturned(gp1::left_trigger) { drive.state = DriveExt.NORMAL }

            /* aim at cone */
            var coneAiming = false
            onPressed(gp1::right_bumper, gp2::right_bumper) {
                coneAiming = !coneAiming
                if (coneAiming) {
                    claw.state = Claw.OPENED
                }
            }
            runIf({ coneAiming }) {
                if (coneDetector.detected) gp1.right_stick_x =
                    conePID.calculate(coneDetector.error).toFloat().also { println("cone aim $it") }

                if (claw.state == Claw.CLOSED) {
                    coneAiming = false
                }
            }

            /* aim at pole */
            var poleAiming = false
            onMoved(gp1::right_trigger, gp2::right_trigger) {
                poleAiming = !poleAiming
            }
            runIf({ poleAiming }) {
                if (poleDetector.detected) gp1.right_stick_x =
                    -polePID.calculate(poleDetector.error)
                        .toFloat()
                        .also { println("pole aim $it") }

                if (claw.state != Claw.CLOSED) {
                    poleAiming = false
                }
            }

            var cycling = false
            onPressed(gp1::dpad_left) {
                cycling = !cycling
                if (arm.state != Arm.GROUND || claw.state != Claw.CLOSED) cycling = false
                if (cycling) {
                    println("GO BACK")
                    drive.followTrajectoryAsync(
                        drive.trajectoryBuilder(Pose2d()).back(32.0).build()
                    )
                    arm.state = Arm.BACKHIGH
                    poleAiming = true
                } else drive.exitTrajectory()
            }
            runIf({ cycling }) { if (!drive.isBusy) cycling = false }


            updates += listOf({ drive.update(gamepads) },
                { arm.update() },
                { tm.addData("retract", odoRetract.state) },
                { tm.update(); Unit },
                {
                    if (drive.state != DriveExt.SPRINTING) {
                        if (arm.state > Arm.STACK && drive.state == DriveExt.NORMAL) drive.state =
                            DriveExt.SLOW
                        if (arm.state <= Arm.STACK && drive.state == DriveExt.SLOW) drive.state =
                            DriveExt.NORMAL
                    }
                },
                { gamepads.sync() },
                { conePID.constants = Triple(pP, pI, pD) },
                { polePID.constants = Triple(jP, jI, jD) })
        }.also {
            waitForStart()
            it.run()
        }
    }

    companion object {
        @JvmField var jP = 0.001
        @JvmField var jI = 0.0
        @JvmField var jD = 0.0

        @JvmField var pP = 0.001
        @JvmField var pI = 0.0
        @JvmField var pD = 0.0
    }
}