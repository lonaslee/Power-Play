package org.firstinspires.ftc.teamcode.teleop

import org.firstinspires.ftc.robotcore.external.Telemetry
import kotlin.reflect.KProperty

typealias Callback = () -> Unit
typealias Predicate = () -> Boolean

class EventLoop(private val running: Predicate, private val telemetry: Telemetry? = null) {
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
     * Execute a [Callback] when a key is pressed.
     */
    fun onPressed(key: KProperty<*>, func: Callback) {
        conditionals += { pressed(key) } to func
    }

    fun onPressed(vararg keys: KProperty<*>, func: Callback) {
        conditionals += { keys.any { pressed(it) } } to func
    }

    /**
     * Execute a [Callback] when a key is released.
     */
    fun onReleased(key: KProperty<*>, func: Callback) {
        conditionals += { released(key) } to func
    }

    fun onReleased(vararg keys: KProperty<*>, func: Callback) {
        conditionals += { keys.any { pressed(it) } } to func
    }

    /**
     * Start the event loop, which will run blocking until the method passed in the constructor returns false.
     */
    fun run() {
        while (running()) {
            val s = System.nanoTime()

            conditionals.filter { it.first() }.forEach { it.second() }
            updates.forEach { it() }

            telemetry?.addData("t", System.nanoTime() - s)
        }
    }
}
