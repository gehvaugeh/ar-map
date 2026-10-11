package com.example.pointcloudmapper.export

import com.example.pointcloudmapper.model.ColoredPoint
import java.io.File
import java.io.FileWriter
import java.io.IOException

/**
 * Exporter for saving point cloud data into ASCII PLY format (.ply)
 * compatible with Blender, CloudCompare, MeshLab, and CAD software.
 */
object PlyExporter {

    /**
     * Exports a list of [ColoredPoint]s to an ASCII .ply file.
     *
     * Format:
     * ply
     * format ascii 1.0
     * comment Created by ARCore Point Cloud Mapper
     * element vertex <count>
     * property float x
     * property float y
     * property float z
     * property uchar red
     * property uchar green
     * property uchar blue
     * end_header
     * <x> <y> <z> <r> <g> <b>
     * ...
     */
    @Throws(IOException::class)
    fun exportToAsciiPly(points: List<ColoredPoint>, outputFile: File) {
        FileWriter(outputFile).use { writer ->
            writer.write("ply\n")
            writer.write("format ascii 1.0\n")
            writer.write("comment Created by ARCore Point Cloud Mapper\n")
            writer.write("element vertex ${points.size}\n")
            writer.write("property float x\n")
            writer.write("property float y\n")
            writer.write("property float z\n")
            writer.write("property uchar red\n")
            writer.write("property uchar green\n")
            writer.write("property uchar blue\n")
            writer.write("end_header\n")

            for (point in points) {
                writer.write("${point.x} ${point.y} ${point.z} ${point.r} ${point.g} ${point.b}\n")
            }
            writer.flush()
        }
    }
}
