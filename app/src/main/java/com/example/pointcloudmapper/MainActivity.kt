package com.example.pointcloudmapper

import android.Manifest
import android.content.Context
import android.content.Intent
import android.content.pm.PackageManager
import android.opengl.GLES20
import android.opengl.GLSurfaceView
import android.os.Bundle
import android.util.Log
import android.view.WindowManager
import android.widget.Button
import android.widget.TextView
import android.widget.Toast
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import androidx.core.content.FileProvider
import com.example.pointcloudmapper.color.ColorProjectionUtils
import com.example.pointcloudmapper.export.PlyExporter
import com.example.pointcloudmapper.rendering.BackgroundRenderer
import com.example.pointcloudmapper.rendering.PointCloudRenderer
import com.example.pointcloudmapper.spatial.VoxelGridPointCloudManager
import com.google.ar.core.Camera
import com.google.ar.core.Config
import com.google.ar.core.Frame
import com.google.ar.core.Session
import com.google.ar.core.exceptions.CameraNotAvailableException
import com.google.ar.core.exceptions.UnavailableException
import java.io.File
import javax.microedition.khronos.egl.EGLConfig
import javax.microedition.khronos.opengles.GL10

class MainActivity : AppCompatActivity(), GLSurfaceView.Renderer {

    private val TAG = "MainActivity"
    private val CAMERA_PERMISSION_CODE = 1001

    private lateinit var surfaceView: GLSurfaceView
    private lateinit var tvPointCount: TextView
    private lateinit var btnClear: Button
    private lateinit var btnExport: Button

    private var session: Session? = null
    private val pointCloudManager = VoxelGridPointCloudManager(voxelSize = 0.03f)

    private val backgroundRenderer = BackgroundRenderer()
    private val pointCloudRenderer = PointCloudRenderer()

    private var viewportWidth = 0
    private var viewportHeight = 0

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        surfaceView = findViewById(R.id.surfaceView)
        tvPointCount = findViewById(R.id.tvPointCount)
        btnClear = findViewById(R.id.btnClear)
        btnExport = findViewById(R.id.btnExport)

        surfaceView.preserveEGLContextOnPause = true
        surfaceView.setEGLContextClientVersion(2)
        surfaceView.setEGLConfigChooser(8, 8, 8, 8, 16, 0)
        surfaceView.setRenderer(this)
        surfaceView.renderMode = GLSurfaceView.RENDERMODE_CONTINUOUSLY

        btnClear.setOnClickListener {
            pointCloudManager.clear()
            updatePointCountUI()
        }

        btnExport.setOnClickListener {
            exportPointCloud()
        }

        checkCameraPermission()
    }

    private fun checkCameraPermission() {
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA)
            != PackageManager.PERMISSION_GRANTED
        ) {
            ActivityCompat.requestPermissions(
                this,
                arrayOf(Manifest.permission.CAMERA),
                CAMERA_PERMISSION_CODE
            )
        }
    }

    override fun onRequestPermissionsResult(
        requestCode: Int,
        permissions: Array<out String>,
        grantResults: IntArray
    ) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
        if (requestCode == CAMERA_PERMISSION_CODE) {
            if (grantResults.isEmpty() || grantResults[0] != PackageManager.PERMISSION_GRANTED) {
                Toast.makeText(
                    this,
                    R.string.camera_permission_required,
                    Toast.LENGTH_LONG
                ).show()
                finish()
            }
        }
    }

    override fun onResume() {
        super.onResume()

        if (session == null && ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA) == PackageManager.PERMISSION_GRANTED) {
            try {
                session = Session(this)
                val config = Config(session)
                config.updateMode = Config.UpdateMode.LATEST_CAMERA_IMAGE

                if (session?.isDepthModeSupported(Config.DepthMode.RAW_DEPTH_ONLY) == true) {
                    config.depthMode = Config.DepthMode.RAW_DEPTH_ONLY
                } else if (session?.isDepthModeSupported(Config.DepthMode.AUTOMATIC) == true) {
                    config.depthMode = Config.DepthMode.AUTOMATIC
                }

                session?.configure(config)
            } catch (e: UnavailableException) {
                Log.e(TAG, "ARCore Unavailable", e)
            }
        }

        try {
            session?.resume()
        } catch (e: CameraNotAvailableException) {
            Log.e(TAG, "Camera not available", e)
            session = null
            return
        }

        surfaceView.onResume()
    }

    override fun onPause() {
        super.onPause()
        surfaceView.onPause()
        session?.pause()
    }

    override fun onDestroy() {
        super.onDestroy()
        session?.close()
        session = null
    }

    // --- GLSurfaceView.Renderer implementation ---

    override fun onSurfaceCreated(gl: GL10?, config: EGLConfig?) {
        GLES20.glClearColor(0.1f, 0.1f, 0.1f, 1.0f)
        backgroundRenderer.createOnGlThread()
        pointCloudRenderer.createOnGlThread()

        session?.let { arSession ->
            arSession.setCameraTextureName(backgroundRenderer.getTextureId())
        }
    }

    override fun onSurfaceChanged(gl: GL10?, width: Int, height: Int) {
        viewportWidth = width
        viewportHeight = height
        GLES20.glViewport(0, 0, width, height)

        val windowManager = getSystemService(Context.WINDOW_SERVICE) as WindowManager
        val displayRotation = windowManager.defaultDisplay.rotation
        session?.setDisplayGeometry(displayRotation, width, height)
    }

    override fun onDrawFrame(gl: GL10?) {
        GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT or GLES20.GL_DEPTH_BUFFER_BIT)

        val currentSession = session ?: return

        try {
            val frame: Frame = currentSession.update()
            val camera: Camera = frame.camera

            backgroundRenderer.draw(frame)

            if (camera.trackingState == com.google.ar.core.TrackingState.TRACKING) {
                val image = try {
                    frame.acquireCameraImage()
                } catch (e: Exception) {
                    null
                }

                if (image != null) {
                    val points = ColorProjectionUtils.extractColoredPoints(
                        frame = frame,
                        camera = camera,
                        yuvImage = image
                    )

                    pointCloudManager.addPoints(points)
                    image.close()

                    runOnUiThread {
                        updatePointCountUI()
                    }
                }
            }

            val projMatrix = FloatArray(16)
            val viewMatrix = FloatArray(16)
            val mvpMatrix = FloatArray(16)

            camera.getProjectionMatrix(projMatrix, 0, 0.1f, 100.0f)
            camera.getViewMatrix(viewMatrix, 0)
            android.opengl.Matrix.multiplyMM(mvpMatrix, 0, projMatrix, 0, viewMatrix, 0)

            val accumulatedPoints = pointCloudManager.getPoints()
            pointCloudRenderer.updatePoints(accumulatedPoints)
            pointCloudRenderer.draw(mvpMatrix)

        } catch (t: Throwable) {
            Log.e(TAG, "Error in onDrawFrame", t)
        }
    }

    private fun updatePointCountUI() {
        val count = pointCloudManager.pointCount()
        tvPointCount.text = getString(R.string.point_count_format, count)
    }

    private fun exportPointCloud() {
        val points = pointCloudManager.getPoints()
        if (points.isEmpty()) {
            Toast.makeText(this, "Point cloud is empty!", Toast.LENGTH_SHORT).show()
            return
        }

        try {
            val exportDir = File(getExternalFilesDir(null), "pointclouds")
            if (!exportDir.exists()) {
                exportDir.mkdirs()
            }

            val file = File(exportDir, "pointcloud_${System.currentTimeMillis()}.ply")
            PlyExporter.exportToAsciiPly(points, file)

            val fileUri = FileProvider.getUriForFile(
                this,
                "$packageName.fileprovider",
                file
            )

            val shareIntent = Intent(Intent.ACTION_SEND).apply {
                type = "text/plain"
                putExtra(Intent.EXTRA_STREAM, fileUri)
                addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION)
            }

            startActivity(Intent.createChooser(shareIntent, "Share Point Cloud PLY"))

            Toast.makeText(this, getString(R.string.export_success, file.name), Toast.LENGTH_LONG).show()

        } catch (e: Exception) {
            Log.e(TAG, "Export failed", e)
            Toast.makeText(this, getString(R.string.export_error, e.message), Toast.LENGTH_LONG).show()
        }
    }
}
