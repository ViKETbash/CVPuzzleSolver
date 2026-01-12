#include "puzzle_assembly.h"

#include <libbase/runtime_assert.h>
#include <libbase/stats.h>
#include <libimages/draw.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <unordered_map>
#include <vector>

namespace {

// Board directions: 0=RIGHT, 1=DOWN, 2=LEFT, 3=UP
static constexpr int dx4[4] = {+1, 0, -1, 0};
static constexpr int dy4[4] = {0, +1, 0, -1};

inline int mod4(int v) noexcept {
    v %= 4;
    if (v < 0) v += 4;
    return v;
}

struct SideLink final {
    int objB = -1;
    int sideB = -1;
    float diff = -1.0f;
};

struct PlacementState final {
    int n = 0;
    std::vector<std::array<SideLink, 4>> links; // per object, per side
    std::vector<int> deg;                       // degree per object (count of non-white sides)
};

static void buildSymmetricLinksOrDie(
    const std::vector<std::vector<MatchedSide>>& objMatchedSides,
    PlacementState& st) {

    st.n = static_cast<int>(objMatchedSides.size());
    st.links.assign(static_cast<size_t>(st.n), {SideLink{}, SideLink{}, SideLink{}, SideLink{}});
    st.deg.assign(static_cast<size_t>(st.n), 0);

    for (int objA = 0; objA < st.n; ++objA) {
        rassert(objMatchedSides[objA].size() == 4, 90100001, "Expected 4 sides per object", objA, (int)objMatchedSides[objA].size());
        for (int sA = 0; sA < 4; ++sA) {
            const auto& ms = objMatchedSides[objA][sA];
            if (ms.objB == -1) {
                // White border side
                st.links[static_cast<size_t>(objA)][static_cast<size_t>(sA)] = SideLink{-1, -1, -1.0f};
                continue;
            }

            const int objB = ms.objB;
            const int sB = ms.sideB;

            rassert(objB >= 0 && objB < st.n, 90100002, "Matched objB out of range", objA, sA, objB);
            rassert(sB >= 0 && sB < 4, 90100003, "Matched sideB out of range", objA, sA, sB);

            rassert(objMatchedSides[objB].size() == 4, 90100004, "Expected 4 sides per object (B)", objB);

            const auto& back = objMatchedSides[objB][sB];

            // Symmetry requirement
            const bool ok = (back.objB == objA && back.sideB == sA);
            rassert(ok, 90100005,
                    "Asymmetric match found:",
                    "A=obj" + std::to_string(objA) + " side" + std::to_string(sA) +
                    " -> B=obj" + std::to_string(objB) + " side" + std::to_string(sB) +
                    ", but reverse is B=obj" + std::to_string(objB) + " side" + std::to_string(sB) +
                    " -> obj" + std::to_string(back.objB) + " side" + std::to_string(back.sideB));

            st.links[static_cast<size_t>(objA)][static_cast<size_t>(sA)] = SideLink{objB, sB, ms.differenceBest};
        }
    }

    // degree = number of non-white sides (edges) (symmetry already ensured)
    for (int obj = 0; obj < st.n; ++obj) {
        int d = 0;
        for (int s = 0; s < 4; ++s) {
            if (st.links[static_cast<size_t>(obj)][static_cast<size_t>(s)].objB != -1) d++;
        }
        st.deg[static_cast<size_t>(obj)] = d;
    }
}

static bool tryAssembleFromCorner(
    const PlacementState& st,
    int startObj,
    std::vector<int>& outObjX,
    std::vector<int>& outObjY,
    std::vector<int>& outObjRot,
    int& outW,
    int& outH) {

    const int n = st.n;
    outObjX.assign(static_cast<size_t>(n), std::numeric_limits<int>::min());
    outObjY.assign(static_cast<size_t>(n), std::numeric_limits<int>::min());
    outObjRot.assign(static_cast<size_t>(n), -1);

    // Start must be corner by degree=2
    if (st.deg[static_cast<size_t>(startObj)] != 2) return false;

    // Find its 2 connected sides
    std::vector<int> connSides;
    connSides.reserve(2);
    for (int s = 0; s < 4; ++s) {
        if (st.links[static_cast<size_t>(startObj)][static_cast<size_t>(s)].objB != -1) connSides.push_back(s);
    }
    if (connSides.size() != 2) return false;

    const int a = connSides[0];
    const int b = connSides[1];

    // Must be adjacent
    int sR = -1;
    int sD = -1;
    if (mod4(a + 1) == b) { sR = a; sD = b; }
    else if (mod4(b + 1) == a) { sR = b; sD = a; }
    else return false;

    // Unique orientation for top-left: sR becomes board RIGHT(0), sD becomes board DOWN(1)
    const int rotStart = mod4(0 - sR);

    // Sanity check: sD must map to DOWN
    rassert(mod4(sD + rotStart) == 1, 90100006, "Corner orientation mismatch", startObj, sR, sD, rotStart);

    // BFS placement in grid coordinates (can be negative; we'll shift later)
    std::deque<int> q;

    outObjX[static_cast<size_t>(startObj)] = 0;
    outObjY[static_cast<size_t>(startObj)] = 0;
    outObjRot[static_cast<size_t>(startObj)] = rotStart;
    q.push_back(startObj);

    // Occupancy map: map grid coordinate -> obj
    // We'll use unordered_map with 64-bit key (x,y) packed.
    auto pack = [](int x, int y) -> std::uint64_t {
        const std::uint32_t ux = static_cast<std::uint32_t>(x);
        const std::uint32_t uy = static_cast<std::uint32_t>(y);
        return (static_cast<std::uint64_t>(ux) << 32) | static_cast<std::uint64_t>(uy);
    };

    std::unordered_map<std::uint64_t, int> occ;
    occ.reserve(static_cast<size_t>(n) * 2);
    occ[pack(0, 0)] = startObj;

    int minX = 0, maxX = 0, minY = 0, maxY = 0;
    int placedCount = 1;

    while (!q.empty()) {
        const int objA = q.front();
        q.pop_front();

        const int xA = outObjX[static_cast<size_t>(objA)];
        const int yA = outObjY[static_cast<size_t>(objA)];
        const int rotA = outObjRot[static_cast<size_t>(objA)];
        rassert(rotA >= 0 && rotA < 4, 90100007, "Invalid rotA");

        for (int dir = 0; dir < 4; ++dir) {
            const int sideA = mod4(dir - rotA); // piece side facing board direction dir
            const SideLink link = st.links[static_cast<size_t>(objA)][static_cast<size_t>(sideA)];
            if (link.objB == -1) continue; // boundary

            const int objB = link.objB;
            const int sideB = link.sideB;

            const int xB = xA + dx4[dir];
            const int yB = yA + dy4[dir];

            const int dirOpp = mod4(dir + 2);
            const int rotB = mod4(dirOpp - sideB);

            // Check occupancy consistency
            const std::uint64_t key = pack(xB, yB);
            auto it = occ.find(key);
            if (it == occ.end()) {
                occ[key] = objB;
            } else {
                rassert(it->second == objB, 90100008,
                        "Grid cell conflict: two objects claim same cell",
                        "cell=(" + std::to_string(xB) + "," + std::to_string(yB) + ")",
                        "existing=obj" + std::to_string(it->second),
                        "new=obj" + std::to_string(objB));
            }

            if (outObjRot[static_cast<size_t>(objB)] == -1) {
                outObjX[static_cast<size_t>(objB)] = xB;
                outObjY[static_cast<size_t>(objB)] = yB;
                outObjRot[static_cast<size_t>(objB)] = rotB;
                q.push_back(objB);
                placedCount++;

                minX = std::min(minX, xB);
                maxX = std::max(maxX, xB);
                minY = std::min(minY, yB);
                maxY = std::max(maxY, yB);
            } else {
                // Must be consistent
                rassert(outObjX[static_cast<size_t>(objB)] == xB && outObjY[static_cast<size_t>(objB)] == yB,
                        90100009,
                        "Object position conflict",
                        "obj" + std::to_string(objB),
                        "existing=(" + std::to_string(outObjX[static_cast<size_t>(objB)]) + "," + std::to_string(outObjY[static_cast<size_t>(objB)]) + ")",
                        "new=(" + std::to_string(xB) + "," + std::to_string(yB) + ")");

                rassert(outObjRot[static_cast<size_t>(objB)] == rotB,
                        90100010,
                        "Object rotation conflict",
                        "obj" + std::to_string(objB),
                        "existing_rot=" + std::to_string(outObjRot[static_cast<size_t>(objB)]),
                        "new_rot=" + std::to_string(rotB));
            }
        }
    }

    if (placedCount != n) return false;

    // Shift to make minX=minY=0
    for (int obj = 0; obj < n; ++obj) {
        outObjX[static_cast<size_t>(obj)] -= minX;
        outObjY[static_cast<size_t>(obj)] -= minY;
    }

    outW = (maxX - minX + 1);
    outH = (maxY - minY + 1);

    if (outW * outH != n) return false;

    // Check full rectangular occupancy (no holes)
    std::vector<int> cellObj(static_cast<size_t>(outW) * static_cast<size_t>(outH), -1);
    for (int obj = 0; obj < n; ++obj) {
        const int x = outObjX[static_cast<size_t>(obj)];
        const int y = outObjY[static_cast<size_t>(obj)];
        if (x < 0 || x >= outW || y < 0 || y >= outH) return false;
        const size_t idx = static_cast<size_t>(y) * static_cast<size_t>(outW) + static_cast<size_t>(x);
        if (cellObj[idx] != -1) return false;
        cellObj[idx] = obj;
    }
    for (int v : cellObj) if (v == -1) return false;

    return true;
}

// Corner mapping:
// piece corners are indexed 0..3 such that corner i is intersection of side(i-1) and side(i),
// and side i spans corner i -> corner(i+1) in clockwise order.
// Board corners in that same indexing are:
//   TR = corner0, BR = corner1, BL = corner2, TL = corner3
static inline int pieceCornerFromBoardCorner(int boardCorner, int rot) noexcept {
    return mod4(boardCorner - rot);
}

static float dist2f(point2i a, point2i b) {
    const float dx = float(a.x - b.x);
    const float dy = float(a.y - b.y);
    return std::sqrt(dx * dx + dy * dy);
}

static int medianRounded(const std::vector<float>& v, int fallback) {
    if (v.empty()) return fallback;
    std::vector<float> tmp = v;
    const float m = static_cast<float>(stats::median(tmp));
    const int r = (int)std::lround(std::max(1.0f, m));
    return r;
}

// ------------------- Homography -------------------

struct H3 final {
    // Row-major 3x3
    double a[9]{};
};

static H3 invert3x3OrDie(const H3& M) {
    const double* m = M.a;
    const double a00 = m[0], a01 = m[1], a02 = m[2];
    const double a10 = m[3], a11 = m[4], a12 = m[5];
    const double a20 = m[6], a21 = m[7], a22 = m[8];

    const double c00 =  a11 * a22 - a12 * a21;
    const double c01 = -(a10 * a22 - a12 * a20);
    const double c02 =  a10 * a21 - a11 * a20;

    const double c10 = -(a01 * a22 - a02 * a21);
    const double c11 =  a00 * a22 - a02 * a20;
    const double c12 = -(a00 * a21 - a01 * a20);

    const double c20 =  a01 * a12 - a02 * a11;
    const double c21 = -(a00 * a12 - a02 * a10);
    const double c22 =  a00 * a11 - a01 * a10;

    const double det = a00 * c00 + a01 * c01 + a02 * c02;
    rassert(std::abs(det) > 1e-12, 90100011, "Homography matrix is singular");

    const double invDet = 1.0 / det;

    H3 inv;
    // adjugate = cofactors transposed
    inv.a[0] = c00 * invDet;
    inv.a[1] = c10 * invDet;
    inv.a[2] = c20 * invDet;

    inv.a[3] = c01 * invDet;
    inv.a[4] = c11 * invDet;
    inv.a[5] = c21 * invDet;

    inv.a[6] = c02 * invDet;
    inv.a[7] = c12 * invDet;
    inv.a[8] = c22 * invDet;

    return inv;
}

static void applyH(const H3& H, double x, double y, double& ox, double& oy) {
    const double* m = H.a;
    const double w = m[6] * x + m[7] * y + m[8];
    rassert(std::abs(w) > 1e-12, 90100012, "Invalid homography w");
    ox = (m[0] * x + m[1] * y + m[2]) / w;
    oy = (m[3] * x + m[4] * y + m[5]) / w;
}

static H3 solveHomography4ptOrDie(
    const std::array<point2f, 4>& src,
    const std::array<point2f, 4>& dst) {

    // Unknowns: h00 h01 h02 h10 h11 h12 h20 h21, with h22=1
    double A[8][9] = {}; // augmented matrix [8 x (8+1)]

    for (int i = 0; i < 4; ++i) {
        const double x = src[i].x;
        const double y = src[i].y;
        const double u = dst[i].x;
        const double v = dst[i].y;

        // u row
        {
            double* r = A[2 * i];
            r[0] = x; r[1] = y; r[2] = 1.0;
            r[3] = 0; r[4] = 0; r[5] = 0;
            r[6] = -x * u; r[7] = -y * u;
            r[8] = u;
        }
        // v row
        {
            double* r = A[2 * i + 1];
            r[0] = 0; r[1] = 0; r[2] = 0;
            r[3] = x; r[4] = y; r[5] = 1.0;
            r[6] = -x * v; r[7] = -y * v;
            r[8] = v;
        }
    }

    // Gaussian elimination with partial pivoting
    for (int col = 0; col < 8; ++col) {
        int piv = col;
        double best = std::abs(A[col][col]);
        for (int r = col + 1; r < 8; ++r) {
            const double v = std::abs(A[r][col]);
            if (v > best) {
                best = v;
                piv = r;
            }
        }
        rassert(best > 1e-12, 90100013, "Homography solve failed: singular system");

        if (piv != col) {
            for (int k = col; k < 9; ++k) std::swap(A[piv][k], A[col][k]);
        }

        const double diag = A[col][col];
        for (int k = col; k < 9; ++k) A[col][k] /= diag;

        for (int r = 0; r < 8; ++r) {
            if (r == col) continue;
            const double f = A[r][col];
            if (f == 0.0) continue;
            for (int k = col; k < 9; ++k) A[r][k] -= f * A[col][k];
        }
    }

    double h[8]{};
    for (int i = 0; i < 8; ++i) h[i] = A[i][8];

    H3 H{};
    H.a[0] = h[0]; H.a[1] = h[1]; H.a[2] = h[2];
    H.a[3] = h[3]; H.a[4] = h[4]; H.a[5] = h[5];
    H.a[6] = h[6]; H.a[7] = h[7]; H.a[8] = 1.0;
    return H;
}

// ------------------- Sampling -------------------

static inline uint8_t clamp_u8(int v) {
    if (v < 0) return 0;
    if (v > 255) return 255;
    return static_cast<uint8_t>(v);
}

static void sampleBilinearRGB(const image8u& img, float x, float y, uint8_t& r, uint8_t& g, uint8_t& b) {
    const int W = img.width();
    const int H = img.height();
    rassert(W > 0 && H > 0, 90100014);

    // Clamp to image bounds
    x = std::clamp(x, 0.0f, float(W - 1));
    y = std::clamp(y, 0.0f, float(H - 1));

    const int x0 = (int)std::floor(x);
    const int y0 = (int)std::floor(y);
    const int x1 = std::min(x0 + 1, W - 1);
    const int y1 = std::min(y0 + 1, H - 1);

    const float fx = x - float(x0);
    const float fy = y - float(y0);

    auto at = [&](int yy, int xx, int c) -> float {
        if (img.channels() == 1) {
            return float(img(yy, xx));
        }
        return float(img(yy, xx, c));
    };

    float c00r = at(y0, x0, 0), c00g = at(y0, x0, 1), c00b = at(y0, x0, 2);
    float c10r = at(y0, x1, 0), c10g = at(y0, x1, 1), c10b = at(y0, x1, 2);
    float c01r = at(y1, x0, 0), c01g = at(y1, x0, 1), c01b = at(y1, x0, 2);
    float c11r = at(y1, x1, 0), c11g = at(y1, x1, 1), c11b = at(y1, x1, 2);

    float r0 = c00r * (1 - fx) + c10r * fx;
    float g0 = c00g * (1 - fx) + c10g * fx;
    float b0 = c00b * (1 - fx) + c10b * fx;

    float r1 = c01r * (1 - fx) + c11r * fx;
    float g1 = c01g * (1 - fx) + c11g * fx;
    float b1 = c01b * (1 - fx) + c11b * fx;

    float rf = r0 * (1 - fy) + r1 * fy;
    float gf = g0 * (1 - fy) + g1 * fy;
    float bf = b0 * (1 - fy) + b1 * fy;

    r = clamp_u8((int)std::lround(rf));
    g = clamp_u8((int)std::lround(gf));
    b = clamp_u8((int)std::lround(bf));
}

static bool maskNearestIsObject(const image8u& mask, float x, float y) {
    const int W = mask.width();
    const int H = mask.height();
    const int xi = (int)std::lround(x);
    const int yi = (int)std::lround(y);
    if (xi < 0 || xi >= W || yi < 0 || yi >= H) return false;
    return mask(yi, xi) == 255;
}

} // namespace

PuzzleAssemblyResult assemblePuzzle(
    const std::vector<image8u>& objImages,
    const std::vector<image8u>& objMasks,
    const std::vector<std::vector<point2i>>& objCorners,
    const std::vector<std::vector<MatchedSide>>& objMatchedSides) {

    const int objects_count = static_cast<int>(objImages.size());
    rassert((int)objMasks.size() == objects_count, 90100020);
    rassert((int)objCorners.size() == objects_count, 90100021);
    rassert((int)objMatchedSides.size() == objects_count, 90100022);

    for (int i = 0; i < objects_count; ++i) {
        rassert(objCorners[i].size() == 4, 90100023, "Each object must have 4 corners", i, (int)objCorners[i].size());
    }

    PlacementState st;
    buildSymmetricLinksOrDie(objMatchedSides, st);

    // Find corner candidates by degree==2
    std::vector<int> cornersCandidates;
    for (int obj = 0; obj < objects_count; ++obj) {
        if (st.deg[static_cast<size_t>(obj)] == 2) cornersCandidates.push_back(obj);
    }
    rassert(!cornersCandidates.empty(), 90100024, "No corner candidates found (degree==2)");

    std::vector<int> objX, objY, objRot;
    int W = 0, H = 0;

    bool assembled = false;
    int usedStart = -1;

    for (int startObj : cornersCandidates) {
        if (tryAssembleFromCorner(st, startObj, objX, objY, objRot, W, H)) {
            assembled = true;
            usedStart = startObj;
            break;
        }
    }

    rassert(assembled, 90100025, "Failed to assemble from any corner candidate");
    (void)usedStart;

    // Build grid
    PuzzleAssemblyResult res;
    res.W = W;
    res.H = H;
    res.grid.assign(W * H, PlacedPiece{-1, 0});

    for (int obj = 0; obj < objects_count; ++obj) {
        const int x = objX[static_cast<size_t>(obj)];
        const int y = objY[static_cast<size_t>(obj)];
        const int rot = objRot[static_cast<size_t>(obj)];
        rassert(rot >= 0 && rot < 4, 90100026);

        const size_t idx = y * W + x;
        rassert(res.grid[idx].obj == -1, 90100027, "Cell already filled", x, y);
        res.grid[idx] = PlacedPiece{obj, rot};
    }

    // 9) TODO Определим ширину/высоту каждого столбика/строки пазла (медиана от ширин/высот назначенных кусочков)
    //    пока что в коде сделано наивно - везде ширина и толщина берется за 200 пикселей
    // Подсказки:
    // 1) сначала подумайте - что нам нужно выяснить из кода выше?
    // 1.1) надо суметь пробежаться по регулярной решетке (переменная res.grid) по столбику и посмотреть какие куски-объекты в нем лежат
    // 1.2) у этих кусков-объектов в столбике надо учесть их количество поворотов на 90 градусов (таких что side0 смотрит направо, side1 - вниз и т.д.)
    // 2) а кто как индексируется в res.grid? можно выяснить посмотрев обращения к ней - нажмите правой кнопкой по ней -> Find Usages (или просто через Ctrl+F)
    // 3) а что хранится в каждом элементе res.grid? тоже можно выяснить по аналогии - по примеру кода который туда записывал данные
    // 4) а куда записать размер для каждого столбика? вот сюда (сейчас сюда пишется W штук чисел 200, где W - число столбиков):
    res.colW = std::vector<int>(W, 200);
    // 5) а куда записать размер для каждой строки? вот сюда (сейчас сюда пишется H штук чисел 200, где H - число строчек):
    res.rowH = std::vector<int>(H, 200);

    // Prefix sums
    std::vector<int> xOff(static_cast<size_t>(W + 1), 0);
    std::vector<int> yOff(static_cast<size_t>(H + 1), 0);
    for (int x = 0; x < W; ++x) xOff[static_cast<size_t>(x + 1)] = xOff[static_cast<size_t>(x)] + res.colW[static_cast<size_t>(x)];
    for (int y = 0; y < H; ++y) yOff[static_cast<size_t>(y + 1)] = yOff[static_cast<size_t>(y)] + res.rowH[static_cast<size_t>(y)];

    const int canvasW = xOff[static_cast<size_t>(W)];
    const int canvasH = yOff[static_cast<size_t>(H)];

    // Assemble with inverse warping per cell
    res.assembled = image8u(canvasW, canvasH, 3);
    res.assembled.fill(0);

    for (int gy = 0; gy < H; ++gy) {
        for (int gx = 0; gx < W; ++gx) {
            const PlacedPiece pp = res.grid[gy * W + gx];
            const int obj = pp.obj;
            const int rot = pp.rot90;

            const image8u& srcImg = objImages[static_cast<size_t>(obj)];
            const image8u& srcMask = objMasks[static_cast<size_t>(obj)];
            const auto& corners = objCorners[static_cast<size_t>(obj)];

            // Destination rectangle corners
            const float X0 = (float)xOff[static_cast<size_t>(gx)];
            const float Y0 = (float)yOff[static_cast<size_t>(gy)];
            const float X1 = (float)xOff[static_cast<size_t>(gx + 1)];
            const float Y1 = (float)yOff[static_cast<size_t>(gy + 1)];

            std::array<point2f, 4> dst = {
                point2f{X0, Y0}, // TL
                point2f{X1, Y0}, // TR
                point2f{X1, Y1}, // BR
                point2f{X0, Y1}  // BL
            };

            // Source corners for these board corners, using rot
            const point2i TLi = corners[static_cast<size_t>(pieceCornerFromBoardCorner(3, rot))];
            const point2i TRi = corners[static_cast<size_t>(pieceCornerFromBoardCorner(0, rot))];
            const point2i BRi = corners[static_cast<size_t>(pieceCornerFromBoardCorner(1, rot))];
            const point2i BLi = corners[static_cast<size_t>(pieceCornerFromBoardCorner(2, rot))];

            std::array<point2f, 4> src = {
                point2f{(float)TLi.x, (float)TLi.y},
                point2f{(float)TRi.x, (float)TRi.y},
                point2f{(float)BRi.x, (float)BRi.y},
                point2f{(float)BLi.x, (float)BLi.y}
            };

            const H3 Hsrc2dst = solveHomography4ptOrDie(src, dst);
            const H3 Hdst2src = invert3x3OrDie(Hsrc2dst);

            const int ix0 = (int)std::floor(std::min(X0, X1));
            const int iy0 = (int)std::floor(std::min(Y0, Y1));
            const int ix1 = (int)std::ceil (std::max(X0, X1));
            const int iy1 = (int)std::ceil (std::max(Y0, Y1));

            for (int Y = iy0; Y < iy1; ++Y) {
                if (Y < 0 || Y >= res.assembled.height()) continue;
                for (int X = ix0; X < ix1; ++X) {
                    if (X < 0 || X >= res.assembled.width()) continue;

                    // Use pixel center
                    double sx = 0.0, sy = 0.0;
                    applyH(Hdst2src, (double)X + 0.5, (double)Y + 0.5, sx, sy);

                    if (!maskNearestIsObject(srcMask, (float)sx, (float)sy)) continue;

                    uint8_t rr = 0, gg = 0, bb = 0;
                    sampleBilinearRGB(srcImg, (float)sx, (float)sy, rr, gg, bb);

                    res.assembled(Y, X, 0) = rr;
                    res.assembled(Y, X, 1) = gg;
                    res.assembled(Y, X, 2) = bb;
                }
            }
        }
    }

    // Add grid lines
    res.assembledWithLines = res.assembled;
    const int thickness = 3;
    const color8u lineColor(0, 255, 0);

    // Vertical lines
    for (int x = 1; x < W; ++x) {
        const int X = xOff[static_cast<size_t>(x)];
        drawSegment(res.assembledWithLines, point2i{X, 0}, point2i{X, canvasH - 1}, lineColor, thickness);
    }
    // Horizontal lines
    for (int y = 1; y < H; ++y) {
        const int Y = yOff[static_cast<size_t>(y)];
        drawSegment(res.assembledWithLines, point2i{0, Y}, point2i{canvasW - 1, Y}, lineColor, thickness);
    }

    return res;
}

void printGrid(std::ostream& os, const PuzzleAssemblyResult& r) {
    os << "Puzzle grid: W=" << r.W << " H=" << r.H << "\n";
    for (int y = 0; y < r.H; ++y) {
        for (int x = 0; x < r.W; ++x) {
            const auto pp = r.grid[y * r.W + x];
            rassert(pp.obj >= 0, 90100030);
            os << "( obj" << pp.obj << " " << pp.rot90 << "x90 rot )";
            if (x + 1 != r.W) os << " ";
        }
        os << "\n";
    }
}
