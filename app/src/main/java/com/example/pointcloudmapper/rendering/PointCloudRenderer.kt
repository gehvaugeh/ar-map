package com.example.pointcloudmapper.rendering

import android.opengl.GLES20
import com.example.pointcloudmapper.model.ColoredPoint
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.nio.FloatBuffer

/**
 * OpenGL ES 2.0 Renderer for rendering colored 3D point cloud overlay.
 * Uses persistent direct buffers to avoid GC thrashing on GL frames.
 */
class PointCloudRenderer {

    private val vertexShaderCode =
        "uniform mat4 u_ModelViewProjection;" +
        "attribute vec4 a_Position;" +
        "attribute vec4 a_Color;" +
        "varying vec4 v_Color;" +
        "void main() {" +
        "   v_Color = a_Color;" +
        "   gl_Position = u_ModelViewProjection * a_Position;" +
        "   gl_PointSize = 8.0;" +
        "}"

    private val fragmentShaderCode =
        "precision mediump float;" +
        "varying vec4 v_Color;" +
        "void main() {" +
        "   gl_FragColor = v_Color;" +
        "}"

    private var program = 0
    private var positionHandle = 0
    private var colorHandle = 0
    private var mvpMatrixHandle = 0

    private var vertexBufferCapacity = 1000
    private var vertexBuffer: FloatBuffer = ByteBuffer.allocateDirect(vertexBufferCapacity * 3 * 4)
        .order(ByteOrder.nativeOrder())
        .asFloatBuffer()

    private var colorBuffer: FloatBuffer = ByteBuffer.allocateDirect(vertexBufferCapacity * 4 * 4)
        .order(ByteOrder.nativeOrder())
        .asFloatBuffer()

    private var pointCount = 0

    fun createOnGlThread() {
        val vertexShader = loadShader(GLES20.GL_VERTEX_SHADER, vertexShaderCode)
        val fragmentShader = loadShader(GLES20.GL_FRAGMENT_SHADER, fragmentShaderCode)

        program = GLES20.glCreateProgram().also {
            GLES20.glAttachShader(it, vertexShader)
            GLES20.glAttachShader(it, fragmentShader)
            GLES20.glLinkProgram(it)
        }

        positionHandle = GLES20.glGetAttribLocation(program, "a_Position")
        colorHandle = GLES20.glGetAttribLocation(program, "a_Color")
        mvpMatrixHandle = GLES20.glGetUniformLocation(program, "u_ModelViewProjection")
    }

    /**
     * Updates point cloud vertex and color buffers from list of ColoredPoint without allocating new buffers every frame.
     */
    fun updatePoints(points: List<ColoredPoint>) {
        pointCount = points.size
        if (pointCount == 0) return

        if (pointCount > vertexBufferCapacity) {
            vertexBufferCapacity = (pointCount * 1.5).toInt()
            vertexBuffer = ByteBuffer.allocateDirect(vertexBufferCapacity * 3 * 4)
                .order(ByteOrder.nativeOrder())
                .asFloatBuffer()
            colorBuffer = ByteBuffer.allocateDirect(vertexBufferCapacity * 4 * 4)
                .order(ByteOrder.nativeOrder())
                .asFloatBuffer()
        }

        vertexBuffer.clear()
        colorBuffer.clear()

        for (i in points.indices) {
            val p = points[i]
            vertexBuffer.put(p.x)
            vertexBuffer.put(p.y)
            vertexBuffer.put(p.z)

            colorBuffer.put(p.r / 255.0f)
            colorBuffer.put(p.g / 255.0f)
            colorBuffer.put(p.b / 255.0f)
            colorBuffer.put(1.0f)
        }

        vertexBuffer.position(0)
        colorBuffer.position(0)
    }

    /**
     * Draws the point cloud with Model-View-Projection matrix.
     */
    fun draw(modelViewProjectionMatrix: FloatArray) {
        if (pointCount == 0) return

        GLES20.glUseProgram(program)

        vertexBuffer.position(0)
        GLES20.glEnableVertexAttribArray(positionHandle)
        GLES20.glVertexAttribPointer(
            positionHandle, 3, GLES20.GL_FLOAT, false, 0, vertexBuffer
        )

        colorBuffer.position(0)
        GLES20.glEnableVertexAttribArray(colorHandle)
        GLES20.glVertexAttribPointer(
            colorHandle, 4, GLES20.GL_FLOAT, false, 0, colorBuffer
        )

        GLES20.glUniformMatrix4fv(mvpMatrixHandle, 1, false, modelViewProjectionMatrix, 0)

        GLES20.glDrawArrays(GLES20.GL_POINTS, 0, pointCount)

        GLES20.glDisableVertexAttribArray(positionHandle)
        GLES20.glDisableVertexAttribArray(colorHandle)
    }

    private fun loadShader(type: Int, shaderCode: String): Int {
        return GLES20.glCreateShader(type).also { shader ->
            GLES20.glShaderSource(shader, shaderCode)
            GLES20.glCompileShader(shader)
        }
    }
}
