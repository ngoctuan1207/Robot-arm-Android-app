package com.example.RobotArm

import android.util.Log
import kotlin.math.*

class LinearPlanner(
    private val ik: RobotIK,
    private val fk: RobotFK,
    private val verbose: Boolean = false,
    ) {
    fun roundIfTiny(v: Float) = if (abs(v) < 1e-3f) 0f else (round(v * 100000) / 100000.0f)
    fun Path_sampling(
        T_start: Array<FloatArray>,
        T_end: Array<FloatArray>,
        step_mm: Float,
        keepOrientation: Boolean
    ): List<Array<FloatArray>> {

        require(step_mm > 0f) { "step_mm must be > 0" }

        // 1) Lấy vị trí XYZ
        val p0 = floatArrayOf(T_start[0][3], T_start[1][3], T_start[2][3])
        val p1 = floatArrayOf(T_end[0][3], T_end[1][3], T_end[2][3])

        // 2) Tính khoảng cách
        val dx = p1[0] - p0[0]
        val dy = p1[1] - p0[1]
        val dz = p1[2] - p0[2]

        val dist = roundIfTiny(sqrt(dx*dx + dy*dy + dz*dz))

        // Nếu không di chuyển → trả đúng 1 điểm
        if (dist < 1e-6f) return listOf(T_start)

        // 3) Tính số steps theo mm
        val steps = max(2, ceil(dist / step_mm).toInt())
        if(verbose) Log.i("LinearPlanner","Step = $steps, distance = $dist")

        return planLinearPath(T_start, T_end, steps, keepOrientation)
    }

    fun planLinearPath(
        T_start: Array<FloatArray>,
        T_end: Array<FloatArray>,
        steps: Int,
        keepOrientation: Boolean
    ): List<Array<FloatArray>> {
        val path = mutableListOf<Array<FloatArray>>()
        val p0 = floatArrayOf(T_start[0][3], T_start[1][3], T_start[2][3])
        val p1 = floatArrayOf(T_end[0][3], T_end[1][3], T_end[2][3])

        val R0 = Array(3) { FloatArray(3) }
        val R1 = Array(3) { FloatArray(3) }

        for (i in 0..2) {
            for (j in 0..2) {
                R0[i][j] = T_start[i][j]
                R1[i][j] = T_end[i][j]
            }
        }

        for (i in 0 until steps) {
            val t = i.toFloat() / (steps - 1)
            val Ti = Array(4) { FloatArray(4) }

            val px = roundIfTiny(p0[0] + (p1[0] - p0[0]) * t)
            val py = roundIfTiny(p0[1] + (p1[1] - p0[1]) * t)
            val pz = roundIfTiny(p0[2] + (p1[2] - p0[2]) * t)
            if (verbose) Log.i("LinearPlanner","percent=${t*100.0f}%       , p[$i] = [$px,$py,$pz]")

            // rotation
            if (keepOrientation) {
//                if (verbose) Log.i("LinearPlanner","Keeping same Orientation")
                for (r in 0..2) {
                    for (c in 0..2) {
                        Ti[r][c] = R0[r][c]
                    }
                }
            }
            else {
//                if (verbose) Log.i("LinearPlanner","Using diff Orientation")
                for (r in 0..2) {
                    for (c in 0..2) {
                        Ti[r][c] = R0[r][c] + (R1[r][c] - R0[r][c]) * t
                    }
                }
                orthonormalize(Ti)
            }

            // translation
            Ti[0][3] = px
            Ti[1][3] = py
            Ti[2][3] = pz

            Ti[3][0] = 0f
            Ti[3][1] = 0f
            Ti[3][2] = 0f
            Ti[3][3] = 1f

            path.add(Ti)
        }

        return path
    }

    private fun orthonormalize(T: Array<FloatArray>) {
        fun norm(v: FloatArray): Float {
            return sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])
        }
        fun normalize(v: FloatArray): FloatArray {
            val n = norm(v)
            if (n < 1e-6f) return v
            return floatArrayOf(v[0]/n, v[1]/n, v[2]/n)
        }

        val x = floatArrayOf(T[0][0], T[1][0], T[2][0])
        val y = floatArrayOf(T[0][1], T[1][1], T[2][1])

        val xn = normalize(x)

        val dotyx = y[0]*xn[0] + y[1]*xn[1] + y[2]*xn[2]
        val yproj = floatArrayOf(dotyx*xn[0], dotyx*xn[1], dotyx*xn[2])
        val yn = normalize(
            floatArrayOf(
                y[0]-yproj[0],
                y[1]-yproj[1],
                y[2]-yproj[2]
            )
        )

        val zn = floatArrayOf(
            xn[1]*yn[2] - xn[2]*yn[1],
            xn[2]*yn[0] - xn[0]*yn[2],
            xn[0]*yn[1] - xn[1]*yn[0]
        )

        for (i in 0..2) {
            T[i][0] = xn[i]
            T[i][1] = yn[i]
            T[i][2] = zn[i]
        }
    }

    fun printPath(path: List<Array<FloatArray>>) {
        for ((i, T) in path.withIndex()) {
            println("===== PATH POINT #$i =====")

            for (r in 0..3) {
                println(T[r].joinToString("\t") { v -> "%.4f".format(v) })
            }
            println() // dòng trống cho dễ đọc
        }
    }

    fun planLinearByMM(
        T_start: Array<FloatArray>,
        T_end: Array<FloatArray>,
        step_mm: Float,
        keepOrientation: Boolean,
        prev: FloatArray
    ): List<FloatArray> {

        val path = Path_sampling(T_start, T_end, step_mm, keepOrientation)
        val jointTrajectory = mutableListOf<FloatArray>()

        var prevSol = prev.copyOf()

        for ((index, pose) in path.withIndex()) {
            if (verbose) {
                println("============================PlanLinearByMM================================")
                Log.i("planLinearByMM","============================Step $index ========================================")
                for (i in 0..3) {
                    println(pose[i].joinToString("\t") { "%.3f".format(it) })
                }
                val Ori = fk.getTCPOrientation(pose)
                val euler = fk.getTCPEulerZYX(Ori)
                println("Euler ZYX = (Ez=%.3f, Ey=%.3f, Ex=%.3f)".format(euler[0], euler[1], euler[2]))
                println("========================================================================")
            }

            val sols = ik.solveIK(pose)
//            sols.forEachIndexed { i, sol ->
//                if (verbose) Log.i("planLinearByMM", "Sol before verified #$i: ${sol.joinToString { "%.2f°".format(it)}} ")
//            }

            val validSols = ik.getVerifiedSolutions(sols, pose)
            validSols.forEachIndexed { i, valid ->
                if (verbose) Log.i("planLinearByMM", "valid Sol #$i: ${valid.joinToString { "%.2f°".format(it)}} ")

            }
            if (validSols.isEmpty()) {
                Log.e("planLinearByMM", "No IK solution for this pose!")
                for (i in 0..3) {
                    if (verbose) println(pose[i].joinToString("\t") { "%.3f".format(it) })
                }
                break
            }

            else{
                val best = pickClosestSolution(validSols, prevSol)
                if (best != null) {
                    jointTrajectory.add(best)

                    // 4) Cập nhật prevSol liên tục
                    prevSol = best.copyOf()
                }
            }
        }

        return jointTrajectory
    }

    fun incLinearByMM(
        T_start: Array<FloatArray>,
        T_end: Array<FloatArray>,
        keepOrientation: Boolean,
        prev: FloatArray
    ): FloatArray {

        var jointTrajectory = FloatArray(7)

        var prevSol = prev.copyOf()

        if (verbose) {
            println("============================incLinearByMM================================")
            for (i in 0..3) {
                println(T_end[i].joinToString("\t") { "%.3f".format(it) })
            }
            val Ori = fk.getTCPOrientation(T_end)
            val euler = fk.getTCPEulerZYX(Ori)
            println("Euler ZYX = (Ez=%.3f, Ey=%.3f, Ex=%.3f)".format(euler[0], euler[1], euler[2]))
            println("========================================================================")
        }

        val sols = ik.solveIK(T_end)
        val validSols = ik.getVerifiedSolutions(sols, T_end)
        if (verbose){
            validSols.forEachIndexed { i, valid ->
                Log.i("planLinearByMM", "valid Sol #$i: ${valid.joinToString { "%.2f°".format(it)}} ")
            }
        }

        if (validSols.isEmpty()) {
            Log.e("planLinearByMM", "No IK solution for this pose!")
            if (verbose){
                for (i in 0..3) {
                    println(T_end[i].joinToString("\t") { "%.3f".format(it) })
                }
            }
        }

        else{
            val best = pickClosestSolution(validSols, prevSol)
            if (best != null) {
                jointTrajectory = best.copyOf()

//                prevSol = best.copyOf()
            }
        }

        return jointTrajectory
    }

    fun Reorient(
        T_start: Array<FloatArray>,
        T_end: Array<FloatArray>,
        prev: FloatArray
    ): List<FloatArray> {

        val jointTrajectory = mutableListOf<FloatArray>()
        var prevSol = prev.copyOf()

        if (verbose) {
            println("===============================Reorient=================================")
            for (i in 0..3) {
                println(T_end[i].joinToString("\t") { "%.3f".format(it) })
            }
            val Ori = fk.getTCPOrientation(T_end)
            val euler = fk.getTCPEulerZYX(Ori)
            println("Euler ZYX = (Ez=%.3f, Ey=%.3f, Ex=%.3f)".format(euler[0], euler[1], euler[2]))
            println("========================================================================")
        }
        val sols = ik.solveIK(T_end)
        val validSols = ik.getVerified_Orient_Solutions(sols, T_end)
        validSols.forEachIndexed { i, valid ->
            if (verbose) Log.i("Reorient", "valid Sol #$i: ${valid.joinToString { "%.2f°".format(it)}} ")
        }
        if (validSols.isEmpty()) {
            Log.e("Reorient", "No IK solution for this Orient pose!")
            for (i in 0..3) {
                if (verbose) println(T_end[i].joinToString("\t") { "%.3f".format(it) })
            }
        }

        else{
            val best = pickClosestSolution(validSols, prevSol)
            if (best != null) {
                jointTrajectory.add(best)
                prevSol = best.copyOf()
            }
        }
        return jointTrajectory
    }


    fun planReorientTrajectory(
        fk: RobotFK,
        ik: RobotIK,
        T_start: Array<FloatArray>,
        T_end: Array<FloatArray>,
        prev: FloatArray,
        steps: Int = 20
    ): List<FloatArray> {

        val trajectory = mutableListOf<FloatArray>()
        var prevSol = prev.copyOf()

        // --- giữ nguyên position ---
        val pos = fk.getTCPPosition(T_start)

        // --- orientation start & end ---
        val R_start = fk.getTCPOrientation(T_start)
        val R_end   = fk.getTCPOrientation(T_end)

        for (i in 1..steps) {

            val t = i.toFloat() / steps

            val R_interp = interpolateRotationSO3(R_start, R_end, t)

            val T_interp = arrayOf(
                floatArrayOf(R_interp[0][0], R_interp[0][1], R_interp[0][2], pos[0]),
                floatArrayOf(R_interp[1][0], R_interp[1][1], R_interp[1][2], pos[1]),
                floatArrayOf(R_interp[2][0], R_interp[2][1], R_interp[2][2], pos[2]),
                floatArrayOf(0f, 0f, 0f, 1f)
            )

            val sols = ik.solveIK(T_interp)
            val valid = ik.getVerified_Orient_Solutions(sols, T_interp)

            if (valid.isEmpty()) {
                Log.e("Planner", "IK failed at step $i")
                break
            }
            val best = pickClosestSolution(valid, prevSol)
                ?: break

            trajectory.add(best)
            prevSol = best.copyOf()
        }

        return trajectory
    }

    fun interpolateRotationSO3(
        R0: Array<FloatArray>,
        R1: Array<FloatArray>,
        t: Float
    ): Array<FloatArray> {

        // R_rel = R0^T * R1
        val R0T = transpose3(R0)
        val R_rel = mul3(R0T, R1)

        // axis-angle từ R_rel
        val (axis, angle) = rotationMatrixToAxisAngle(R_rel)

        // scale góc
        val angle_t = angle * t

        // exp(axis * angle_t)
        val R_inc = axisAngleToRotationMatrix(axis, angle_t)

        // R_interp = R0 * R_inc
        return mul3(R0, R_inc)
    }

    fun rotationMatrixToAxisAngle(
        R: Array<FloatArray>
    ): Pair<FloatArray, Float> {

        val trace = R[0][0] + R[1][1] + R[2][2]
        val angle = acos(((trace - 1f) / 2f).coerceIn(-1f, 1f))

        if (abs(angle) < 1e-6f) {
            return Pair(floatArrayOf(1f, 0f, 0f), 0f)
        }

        val denom = 2f * sin(angle)

        val x = (R[2][1] - R[1][2]) / denom
        val y = (R[0][2] - R[2][0]) / denom
        val z = (R[1][0] - R[0][1]) / denom

        return Pair(floatArrayOf(x, y, z), angle)
    }

    fun axisAngleToRotationMatrix(
        axis: FloatArray,
        angle: Float
    ): Array<FloatArray> {

        val x = axis[0]
        val y = axis[1]
        val z = axis[2]

        val c = cos(angle)
        val s = sin(angle)
        val v = 1f - c

        return arrayOf(
            floatArrayOf(
                c + x*x*v,
                x*y*v - z*s,
                x*z*v + y*s
            ),
            floatArrayOf(
                y*x*v + z*s,
                c + y*y*v,
                y*z*v - x*s
            ),
            floatArrayOf(
                z*x*v - y*s,
                z*y*v + x*s,
                c + z*z*v
            )
        )
    }

    fun pickClosestSolution(
        candidates: List<FloatArray>,
        prev: FloatArray?
    ): FloatArray? {
        if (candidates.isEmpty()) return null
        if (prev == null) return candidates[0]

        var best: FloatArray? = null
        var bestScore = Float.MAX_VALUE

        for (sol in candidates) {
            var s = 0f
            for (i in 1..6) {
                val d = sol[i] - prev[i]
                s += d * d
            }

            if (s < bestScore) {
                bestScore = s
                best = sol
            }
        }

        return best
    }

    private fun mul3(A: Array<FloatArray>, B: Array<FloatArray>): Array<FloatArray> {
        val R = Array(3) { FloatArray(3) }
        for (i in 0..2) {
            for (j in 0..2) {
                var s = 0f
                for (k in 0..2) {
                    s += A[i][k] * B[k][j]
                }
                R[i][j] = s
            }
        }
        return R
    }

    private fun transpose3(m: Array<FloatArray>): Array<FloatArray> {
        return arrayOf(
            floatArrayOf(m[0][0], m[1][0], m[2][0]),
            floatArrayOf(m[0][1], m[1][1], m[2][1]),
            floatArrayOf(m[0][2], m[1][2], m[2][2])
        )
    }

}
