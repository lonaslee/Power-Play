package org.firstinspires.ftc.teamcode.teleop

import com.outoftheboxrobotics.photoncore.PhotonCore
import com.qualcomm.hardware.lynx.LynxModule
import org.firstinspires.ftc.robotcore.external.Telemetry
import kotlin.reflect.KProperty
import kotlin.system.measureNanoTime

typealias Callback = () -> Unit
typealias Predicate = () -> Boolean

class EventLoop(private val running: Predicate, private val telemetry: Telemetry? = null) {
    init {
        PhotonCore.CONTROL_HUB.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        PhotonCore.EXPANSION_HUB.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        PhotonCore.enable()
    }

    /**
     * [Callback]s that need to be executed on every loop.
     */
    val updates = mutableListOf<Callback>()

    private val conditionals = mutableListOf<Pair<Predicate, Callback>>()

    /**
     * Execute a [Callback] if the [Predicate] evaluates to true every loop.
     */
    fun runIf(pred: Predicate, func: Callback) {
        conditionals += pred to func
    }

    /**
     * Execute a [Callback] when a key is [pressed].
     */
    fun onPressed(key: KProperty<*>, func: Callback) {
        conditionals += { pressed(key) } to func
    }

    fun onPressed(vararg keys: KProperty<*>, func: Callback) {
        conditionals += { keys.any { pressed(it) } } to func
    }

    /**
     * Execute a [Callback] when a key is [released].
     */
    fun onReleased(key: KProperty<*>, func: Callback) {
        conditionals += { released(key) } to func
    }

    fun onReleased(vararg keys: KProperty<*>, func: Callback) {
        conditionals += { keys.any { pressed(it) } } to func
    }

    /**
     * Execute a [Callback] when a trigger or joystick axis has been [moved].
     */
    fun onMoved(key: KProperty<*>, func: Callback) {
        conditionals += { moved(key) } to func
    }

    fun onMoved(vararg keys: KProperty<*>, func: Callback) {
        conditionals += { keys.any { moved(it) } } to func
    }

    /**
     * Start the event loop, which will run blocking until the method passed in the constructor returns false.
     */
    fun run() {

        var time = 0L
        while (running()) time = measureNanoTime {
            conditionals.filter { it.first() }.forEach { it.second() }
            updates.forEach { it() }

            telemetry?.addData("hz", 1e9 / time)
            PhotonCore.CONTROL_HUB.clearBulkCache()
            PhotonCore.EXPANSION_HUB.clearBulkCache()
        }
    }
}
