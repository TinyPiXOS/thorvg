/*
 * Copyright (c) 2020 - 2025 the ThorVG project. All rights reserved.

 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "tvgGlCommon.h"
#include "tvgGlGpuBuffer.h"
#include "tvgGlTessellator.h"
#include "tvgGlRenderTask.h"

#if 1
bool GlGeometry::intersects(const RenderRegion& region) const
{
    if (region.min.x < 0 || region.min.y < 0) return false;

    Matrix im;
    if (!inverse(&matrix, &im)) return false;
    auto v = Point{(float)region.min.x, (float)region.min.y} * im;

    auto hit = [&](const float* pt1, const float* pt2, const float* pt3, const Point& pt) -> bool {
        Point v0 = {pt3[0] - pt1[0], pt3[1] - pt1[1]};
        Point v1 = {pt2[0] - pt1[0], pt2[1] - pt1[1]};
        Point v2 = {pt.x - pt1[0], pt.y - pt1[1]};

        // Compute dot products
        auto dot00 = tvg::dot(v0, v0);
        auto dot01 = tvg::dot(v0, v1);
        auto dot02 = tvg::dot(v0, v2);
        auto dot11 = tvg::dot(v1, v1);
        auto dot12 = tvg::dot(v1, v2);

        // Compute barycentric coordinates
        auto denom = dot00 * dot11 - dot01 * dot01;
        if (tvg::zero(denom)) return false; // Degenerate triangle

        auto idenom = 1.0f / denom;
        auto u = (dot11 * dot02 - dot01 * dot12) * idenom;
        auto v = (dot00 * dot12 - dot01 * dot02) * idenom;

        // Check if point is in triangle
        return (u >= 0.0f && v >= 0.0f && (u + v) <= 1.0f);
    };

    auto odd = false;  //even odd tessllation

    //stroke
    for (uint32_t i = 0; i < stroke.index.count; i += 3) {
        if (hit(&stroke.vertex[stroke.index[i] * 2], &stroke.vertex[stroke.index[i + 1] * 2], &stroke.vertex[stroke.index[i + 2] * 2], v)) odd = !odd;
    }
    if (odd) return true;

    //fill
    odd = false;
    for (uint32_t i = 0; i < fill.index.count; i += 3) {
        if (hit(&fill.vertex[fill.index[i] * 2], &fill.vertex[fill.index[i + 1] * 2], &fill.vertex[fill.index[i + 2] * 2], v)) odd = !odd;
    }
    if (odd) return true;

    return false;
}
#else
bool GlGeometry::intersects(const RenderRegion& region) const
{
    Matrix im;
    if (!inverse(&matrix, &im)) return false;

    Point edge[4] = {
        {(float)region.min.x, (float)region.min.y},
        {(float)region.max.x, (float)region.min.y},
        {(float)region.max.x, (float)region.max.y},
        {(float)region.min.x, (float)region.max.y}
    };

    for (int i = 0; i < 4; ++i) {
        edge[i] *= im;
    }

    auto hit = [&](const float* pt1, const float* pt2, const float* pt3, const Point* edge) -> bool {

        const Point poly[3] = {{pt1[0], pt1[1]}, {pt2[0], pt2[1]}, {pt3[0], pt3[1]}};

        // projection of polygon onto axis
        auto project = [&](const Point* poly, int count, const Point& axis, float& min, float& max) {
            min = max = tvg::dot(poly[0], axis);
            for (int i = 1; i < count; ++i) {
                auto d = tvg::dot(poly[i], axis);
                if (d < min) min = d;
                else if (d > max) max = d;
            }
        };

        // check overlap on axis
        auto overlap = [&](const Point& axis) -> bool {
            float minA, maxA, minB, maxB;
            project(poly, 3, axis, minA, maxA);
            project(edge, 4, axis, minB, maxB);
            return !(maxA < minB || maxB < minA);
        };

        // polygon edge detection
        for (int i = 0; i < 3; ++i) {
            auto vedge = poly[(i + 1) % 3] - poly[i];
            Point axis = {-vedge.y, vedge.x}; // normal vector
            tvg::normalize(axis);
            if (!overlap(axis)) return false;
        }

        // rect edge detection
        auto r1 = edge[1] - edge[0];
        auto r2 = edge[3] - edge[0];

        Point axis1 = {-r1.y, r1.x};
        tvg::normalize(axis1);

        Point axis2 = {-r2.y, r2.x};
        tvg::normalize(axis2);

        if (!overlap(axis1)) return false;
        if (!overlap(axis2)) return false;

        return true;
    };

    auto odd = false;  //even odd tessllation

    //stroke
    for (uint32_t i = 0; i < stroke.index.count; i += 3) {
        if (hit(&stroke.vertex[stroke.index[i] * 2], &stroke.vertex[stroke.index[i + 1] * 2], &stroke.vertex[stroke.index[i + 2] * 2], edge)) return true;
    }
    if (odd) return true;

    //fill
    odd = false;
    for (uint32_t i = 0; i < fill.index.count; i += 3) {
        if (hit(&fill.vertex[fill.index[i] * 2], &fill.vertex[fill.index[i + 1] * 2], &fill.vertex[fill.index[i + 2] * 2], edge)) return true;
    }
    if (odd) return true;

    return false;
}
#endif

bool GlGeometry::tesselate(const RenderShape& rshape, RenderUpdateFlag flag)
{
    if (flag & (RenderUpdateFlag::Color | RenderUpdateFlag::Gradient | RenderUpdateFlag::Transform | RenderUpdateFlag::Path)) {
        fill.clear();
        BWTessellator bwTess{&fill};
        if (rshape.trimpath()) {
            RenderPath trimmedPath;
            if (rshape.stroke->trim.trim(rshape.path, trimmedPath)) bwTess.tessellate(trimmedPath, matrix);
            else return true;
        } else {
            bwTess.tessellate(rshape.path, matrix);
        }

        fillRule = rshape.rule;
        bounds = bwTess.bounds();
    }

    if (flag & (RenderUpdateFlag::Stroke | RenderUpdateFlag::GradientStroke | RenderUpdateFlag::Transform)) {
        stroke.clear();
        auto strokeWidth = 0.0f;
        if (isinf(matrix.e11)) {
            strokeWidth = rshape.strokeWidth() * scaling(matrix);
            if (strokeWidth <= MIN_GL_STROKE_WIDTH) strokeWidth = MIN_GL_STROKE_WIDTH;
            strokeWidth = strokeWidth / matrix.e11;
        } else {
            strokeWidth = rshape.strokeWidth();
        }
        //run stroking only if it's valid
        if (!tvg::zero(strokeWidth)) {
            Stroker stroker(&stroke, strokeWidth);
            stroker.run(rshape, matrix);
            bounds = stroker.bounds();
        }
    }
    return true;
}

bool GlGeometry::tesselate(const RenderSurface* image, RenderUpdateFlag flag)
{
    if (!(flag & RenderUpdateFlag::Image)) return true;

    fill.clear();

    fill.vertex.reserve(5 * 4);
    fill.index.reserve(6);

    auto left = 0.f;
    auto top = 0.f;
    auto right = float(image->w);
    auto bottom = float(image->h);

    // left top point
    fill.vertex.push(left);
    fill.vertex.push(top);

    fill.vertex.push(0.f);
    fill.vertex.push(1.f);
    // left bottom point
    fill.vertex.push(left);
    fill.vertex.push(bottom);

    fill.vertex.push(0.f);
    fill.vertex.push(0.f);
    // right top point
    fill.vertex.push(right);
    fill.vertex.push(top);

    fill.vertex.push(1.f);
    fill.vertex.push(1.f);
    // right bottom point
    fill.vertex.push(right);
    fill.vertex.push(bottom);

    fill.vertex.push(1.f);
    fill.vertex.push(0.f);

    fill.index.push(0);
    fill.index.push(1);
    fill.index.push(2);

    fill.index.push(2);
    fill.index.push(1);
    fill.index.push(3);

    bounds = {{0, 0}, {int32_t(image->w), int32_t(image->h)}};

    return true;
}


bool GlGeometry::draw(GlRenderTask* task, GlStageBuffer* gpuBuffer, RenderUpdateFlag flag)
{
    if (flag == RenderUpdateFlag::None) return false;

    auto buffer = ((flag & RenderUpdateFlag::Stroke) || (flag & RenderUpdateFlag::GradientStroke)) ? &stroke : &fill;
    if (buffer->index.empty()) return false;

    auto vertexOffset = gpuBuffer->push(buffer->vertex.data, buffer->vertex.count * sizeof(float));
    auto indexOffset = gpuBuffer->pushIndex(buffer->index.data, buffer->index.count * sizeof(uint32_t));

    // vertex layout
    if (flag & RenderUpdateFlag::Image) {
        // image has two attribute: [pos, uv]
        task->addVertexLayout(GlVertexLayout{0, 2, 4 * sizeof(float), vertexOffset});
        task->addVertexLayout(GlVertexLayout{1, 2, 4 * sizeof(float), vertexOffset + 2 * sizeof(float)});
    } else {
        task->addVertexLayout(GlVertexLayout{0, 2, 2 * sizeof(float), vertexOffset});
    }
    task->setDrawRange(indexOffset, buffer->index.count);
    return true;
}


GlStencilMode GlGeometry::getStencilMode(RenderUpdateFlag flag)
{
    if (flag & RenderUpdateFlag::Stroke) return GlStencilMode::Stroke;
    if (flag & RenderUpdateFlag::GradientStroke) return GlStencilMode::Stroke;
    if (flag & RenderUpdateFlag::Image) return GlStencilMode::None;

    if (fillRule == FillRule::NonZero) return GlStencilMode::FillNonZero;
    if (fillRule == FillRule::EvenOdd) return GlStencilMode::FillEvenOdd;

    return GlStencilMode::None;
}


RenderRegion GlGeometry::getBounds() const
{
    if (tvg::identity(&matrix)) return bounds;

    auto lt = Point{float(bounds.min.x), float(bounds.min.y)} * matrix;
    auto lb = Point{float(bounds.min.x), float(bounds.max.y)} * matrix;
    auto rt = Point{float(bounds.max.x), float(bounds.min.y)} * matrix;
    auto rb = Point{float(bounds.max.x), float(bounds.max.y)} * matrix;

    auto left = min(min(lt.x, lb.x), min(rt.x, rb.x));
    auto top = min(min(lt.y, lb.y), min(rt.y, rb.y));
    auto right = max(max(lt.x, lb.x), max(rt.x, rb.x));
    auto bottom = max(max(lt.y, lb.y), max(rt.y, rb.y));

    auto bounds = RenderRegion {{int32_t(floor(left)), int32_t(floor(top))}, {int32_t(ceil(right)), int32_t(ceil(bottom))}};
    if (bounds.valid()) return bounds;
    return this->bounds;

}