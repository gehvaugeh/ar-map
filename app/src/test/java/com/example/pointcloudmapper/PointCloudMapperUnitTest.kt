package com.example.pointcloudmapper

import com.example.pointcloudmapper.export.PlyExporter
import com.example.pointcloudmapper.model.ColoredPoint
import com.example.pointcloudmapper.spatial.VoxelGridPointCloudManager
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import java.io.File

class PointCloudMapperUnitTest {

    @Test
    fun testVoxelGridDeduplication() {
        val manager = VoxelGridPointCloudManager(voxelSize = 0.05f) // 5cm voxels

        // Point 1 in voxel (0, 0, 0)
        val p1 = ColoredPoint(0.01f, 0.01f, 0.01f, 255, 0, 0, confidence = 0.5f)
        // Point 2 in same voxel (0, 0, 0) with higher confidence
        val p2 = ColoredPoint(0.02f, 0.02f, 0.02f, 0, 255, 0, confidence = 0.9f)
        // Point 3 in different voxel (1, 1, 1) -> coordinates (0.06, 0.06, 0.06)
        val p3 = ColoredPoint(0.06f, 0.06f, 0.06f, 0, 0, 255, confidence = 0.8f)

        manager.addPoint(p1)
        assertEquals(1, manager.pointCount())

        manager.addPoint(p2)
        assertEquals(1, manager.pointCount()) // Replaced in same voxel because p2 has higher confidence
        assertEquals(255, manager.getPoints()[0].g)

        manager.addPoint(p3)
        assertEquals(2, manager.pointCount())
    }

    @Test
    fun testPlyExporterFormat() {
        val tempFile = File.createTempFile("test_pointcloud", ".ply")
        tempFile.deleteOnExit()

        val points = listOf(
            ColoredPoint(1.0f, 2.0f, 3.0f, 255, 128, 64),
            ColoredPoint(-0.5f, 0.0f, 1.2f, 0, 255, 0)
        )

        PlyExporter.exportToAsciiPly(points, tempFile)

        val lines = tempFile.readLines()
        assertEquals("ply", lines[0])
        assertEquals("format ascii 1.0", lines[1])
        assertTrue(lines.contains("element vertex 2"))
        assertTrue(lines.contains("property float x"))
        assertTrue(lines.contains("property float y"))
        assertTrue(lines.contains("property float z"))
        assertTrue(lines.contains("property uchar red"))
        assertTrue(lines.contains("property uchar green"))
        assertTrue(lines.contains("property uchar blue"))
        assertEquals("end_header", lines[lines.indexOf("end_header")])

        val lastTwoLines = lines.takeLast(2)
        assertEquals("1.0 2.0 3.0 255 128 64", lastTwoLines[0])
        assertEquals("-0.5 0.0 1.2 0 255 0", lastTwoLines[1])
    }
}
