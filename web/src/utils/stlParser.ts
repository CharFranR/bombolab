/**
 * Binary STL parser — standalone utility, zero dependencies.
 *
 * Reads little-endian binary STL, validates structural integrity by
 * checking `(fileSize - 84) / 50 === triangleCount`, and returns
 * vertex data, bounding box, and geometric centroid.
 *
 * Not called at runtime by the app — used manually for reference
 * geometry during calibration value authoring (console/script).
 */

export interface BBox {
  min: [number, number, number];
  max: [number, number, number];
}

export interface StlParseResult {
  vertices: Float32Array;
  bbox: BBox;
  centroid: [number, number, number];
}

export function parseStl(buffer: ArrayBuffer): StlParseResult | null {
  // Binary STL: 80-byte header + 4-byte triangle count + N * 50-byte triangles
  const HEADER_SIZE = 80;
  const COUNT_SIZE = 4;
  const TRIANGLE_BYTE_SIZE = 50;

  const fileSize = buffer.byteLength;

  if (fileSize < HEADER_SIZE + COUNT_SIZE) {
    return null;
  }

  const dv = new DataView(buffer);
  const triangleCount = dv.getUint32(HEADER_SIZE, /* littleEndian */ true);

  // Validate: each triangle is exactly 50 bytes (12 floats for normal + 3*12 for vertices + 2 reserved)
  const expectedSize = HEADER_SIZE + COUNT_SIZE + triangleCount * TRIANGLE_BYTE_SIZE;
  if (fileSize !== expectedSize) {
    console.warn(
      `[stlParser] Size mismatch: expected ${expectedSize} bytes for ${triangleCount} triangles, got ${fileSize}`,
    );
    return null;
  }

  // No triangles → empty result
  if (triangleCount === 0) {
    return {
      vertices: new Float32Array(0),
      bbox: {
        min: [0, 0, 0],
        max: [0, 0, 0],
      },
      centroid: [0, 0, 0],
    };
  }

  // Extract vertices (9 floats per triangle: 3 vertices × 3 coords each)
  // Each triangle layout: normal (3 floats, 12 bytes) + vertex1 (3 floats) + vertex2 + vertex3 + attribute (2 bytes)
  const floatsPerTriangle = 9;
  const vertices = new Float32Array(triangleCount * floatsPerTriangle);
  let minX = Infinity, minY = Infinity, minZ = Infinity;
  let maxX = -Infinity, maxY = -Infinity, maxZ = -Infinity;
  let sumX = 0, sumY = 0, sumZ = 0;
  let vertexCount = 0;

  for (let i = 0; i < triangleCount; i++) {
    const baseOffset = HEADER_SIZE + COUNT_SIZE + i * TRIANGLE_BYTE_SIZE;
    // Skip normal (3 floats = 12 bytes), read vertices starting at offset 12
    const vOffset = baseOffset + 12;

    for (let j = 0; j < 3; j++) {
      const vx = dv.getFloat32(vOffset + j * 12, /* littleEndian */ true);
      const vy = dv.getFloat32(vOffset + j * 12 + 4, /* littleEndian */ true);
      const vz = dv.getFloat32(vOffset + j * 12 + 8, /* littleEndian */ true);

      const outIdx = i * floatsPerTriangle + j * 3;
      vertices[outIdx] = vx;
      vertices[outIdx + 1] = vy;
      vertices[outIdx + 2] = vz;

      if (vx < minX) minX = vx;
      if (vy < minY) minY = vy;
      if (vz < minZ) minZ = vz;
      if (vx > maxX) maxX = vx;
      if (vy > maxY) maxY = vy;
      if (vz > maxZ) maxZ = vz;

      sumX += vx;
      sumY += vy;
      sumZ += vz;
      vertexCount++;
    }
  }

  const totalVerts = triangleCount * 3;
  return {
    vertices,
    bbox: {
      min: [minX, minY, minZ],
      max: [maxX, maxY, maxZ],
    },
    centroid: [sumX / totalVerts, sumY / totalVerts, sumZ / totalVerts],
  };
}
