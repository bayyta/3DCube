#include <windows.h>
#undef SIZE
#undef max
#undef min
#undef NEAR
#undef FAR
#undef near
#undef far

#include <iostream>
#include "Eigen/Eigen"
#include <chrono>
#include <thread>
#include <string>
#include <sstream>
#include <vector>
#include <math.h>

void getCube(std::vector<Eigen::Vector4d>& vertices, std::vector<Eigen::Vector3i>& indices);
Eigen::Matrix4d createProjMat(const double& near, const double& far, const double& fov);
void translate(Eigen::Vector4d& point, const Eigen::Vector4d& dist);
void rotate(Eigen::Vector4d& point, const Eigen::Vector4d& origin, double rads, int axis);
Eigen::Vector3d getNormal(const Eigen::Vector4d& p1, const Eigen::Vector4d& p2, const Eigen::Vector4d& p3);
bool inside(const Eigen::Vector2d& p, const Eigen::Vector2d& c1, const Eigen::Vector2d& c2, const Eigen::Vector2d& c3);

struct pix {
    char val;
    double z;
};

int main()
{
    const int WIDTH = 80;
    const int HEIGHT = 40;
    const double NEAR = 0.1;
    const double FAR = 20.0;
    const double FOV = 90.0;

    std::cout << "\e[8;" << HEIGHT << ";" << WIDTH << "t";
    auto projMat = createProjMat(NEAR, FAR, FOV);

    std::vector<Eigen::Vector4d> verts;
    std::vector<Eigen::Vector3i> inds;
    getCube(verts, inds);
    std::vector<Eigen::Vector4d> projVerts(verts);

    // Get center
    Eigen::Vector4d center;
    double _minX = 1e9, _maxX = -1e9, _minY = 1e9, _maxY = -1e9, _minZ = 1e9, _maxZ = -1e9;
    for (auto& vec : verts) {
        _minX = std::min(_minX, vec[0]);
        _maxX = std::max(_maxX, vec[0]);
        _minY = std::min(_minY, vec[1]);
        _maxY = std::max(_maxY, vec[1]);
        _minZ = std::min(_minZ, vec[2]);
        _maxZ = std::max(_maxZ, vec[2]); 
    }
    center << 0.5 * (_maxX + _minX), 0.5 * (_maxY + _minY), 0.5 * (_maxZ + _minZ), 1.0;

    std::vector<std::vector<pix>> pixels(HEIGHT, std::vector<pix>(WIDTH, {0, 1e9}));
    const std::string symbols = "@!;,"; //"@$#*!=;:~-,.";
    double gap = 90.0 / symbols.length();

    HANDLE hConsole_c = GetStdHandle(STD_OUTPUT_HANDLE);
    int _iter = 0;
    while (1) {
        // Clear screen
        for (int y = 0; y < HEIGHT; y++) {
            for (int x = 0; x < WIDTH; x++) {
                pixels[y][x].val = 0.0;
                pixels[y][x].z = 1e9;
            }
        }

        // Rotate and project to image plane
        for (int i = 0; i < projVerts.size(); i++) {
            rotate(verts[i], center, 3.0 * M_PI / 180, 0);
            rotate(verts[i], center, 3.0 * M_PI / 180, 1);
            //rotate(verts[i], center, 3.0 * M_PI / 180, 2);
            projVerts[i] = verts[i].transpose() * projMat;
            projVerts[i] /= projVerts[i][3];
        }

        // Render
        for (int i = 0; i < inds.size(); i++) {
            // Get normal
            auto& vert1 = verts[inds[i][0]];
            auto& vert2 = verts[inds[i][1]];
            auto& vert3 = verts[inds[i][2]];
            auto normal = getNormal(vert1, vert2, vert3);
            auto ray = (vert1.head(3) + vert2.head(3) + vert3.head(3)) / 3;
            double angle = 180.0 * std::acos(ray.dot(normal) / (ray.norm() * normal.norm())) / M_PI;
            angle = std::min(angle, 180.0 - angle);

            char pixSymb = symbols[std::min(int(angle / gap), (int)symbols.length() - 1)];
            
            // Fill pixel coordinates
            Eigen::Vector2d pCoords[3];
            double zVal = 0.0;

            for (int j = 0; j < 3; j++) {
                auto& pvert = projVerts[inds[i][j]];
                zVal += pvert[2] / 3;
                pCoords[j] = {(0.5 * (pvert[0] + 1.0)) * WIDTH, (1.0 - 0.5 * (pvert[1] + 1.0)) * HEIGHT};                
            }

            int startX = (int) std::min(pCoords[0][0], std::min(pCoords[1][0], pCoords[2][0]));
            int startY = (int) std::min(pCoords[0][1], std::min(pCoords[1][1], pCoords[2][1]));
            int endX = (int) std::max(pCoords[0][0], std::max(pCoords[1][0], pCoords[2][0]));
            int endY = (int) std::max(pCoords[0][1], std::max(pCoords[1][1], pCoords[2][1]));

            for (int yc = startY; yc < endY; yc++) {
                for (int xc = startX; xc < endX; xc++) {
                    if (xc < 0 || xc >= WIDTH || yc < 0 || yc >= HEIGHT) continue;
                    if (inside({double(xc), double(yc)}, pCoords[0], pCoords[1], pCoords[2]) && pixels[yc][xc].z > zVal) {
                        pixels[yc][xc].val = pixSymb;
                        pixels[yc][xc].z = zVal;
                    }
                }
            }

        }

        std::stringstream ss;
        ss << '\r';
        for (int y = 0; y < HEIGHT; y++) {
            for (int x = 0; x < WIDTH; x++) {
                if (pixels[y][x].val == 0) ss << ' ';
                else {
                    ss << pixels[y][x].val;
                }
            }
        }

        DWORD len = WIDTH * HEIGHT;
        DWORD dwBytesWritten = 0;
        WriteConsoleOutputCharacter(hConsole_c, ss.str().c_str(), len, {0, 0}, &dwBytesWritten);
        
        //system("clear");
        //std::cout << ss.str() << std::flush;
        //std::string asd;
        //std::cin >> asd;
        _iter++;
        std::this_thread::sleep_for(std::chrono::milliseconds(33));
    }

    CloseHandle(hConsole_c);


    std::string asd;
    std::cin >> asd;
    system("clear");

    return 0;
}

bool inside(const Eigen::Vector2d& p, const Eigen::Vector2d& c1, const Eigen::Vector2d& c2, const Eigen::Vector2d& c3) {
    double as_x = p[0] - c1[0];
    double as_y = p[1] - c1[1];

    bool s_ab = ((c2[0] - c1[0]) * as_y - (c2[1]-c1[1])*as_x) > 0.0;
    if (((c3[0]-c1[0])*as_y-(c3[1]-c1[1])*as_x) > 0 == s_ab) return false;
    if (((c3[0]-c2[0])*(p[1]-c2[1])-(c3[1]-c2[1])*(p[0]-c2[0])) > 0 != s_ab) return false;

    return true;    
}

Eigen::Matrix4d createProjMat(const double& near, const double& far, const double& fov) {
    double S = 1.0 / tan(M_PI * fov / (2*180));
    Eigen::Matrix4d cam = Eigen::Matrix4d::Zero();
    cam(0, 0) = S;
    cam(1, 1) = S;
    cam(2, 2) = -far/(far - near);
    cam(2, 3) = -1.0;
    cam(3, 2) = -far*near/(far - near);

    return cam;
}

Eigen::Vector3d getNormal(const Eigen::Vector4d& p1, const Eigen::Vector4d& p2, const Eigen::Vector4d& p3) {
    auto A = p2 - p1;
    auto B = p3 - p1;

    Eigen::Vector3d normal;
    normal << A[1] * B[2] - A[2] * B[1], A[2] * B[0] - A[0] * B[2], A[0] * B[1] - A[1] * B[0];
    return normal;
}

void translate(Eigen::Vector4d& point, const Eigen::Vector4d& dist) {
    point[0] += dist[0];
    point[1] += dist[1];
    point[2] += dist[2];
}

void rotate(Eigen::Vector4d& point, const Eigen::Vector4d& origin, double rads, int axis) {
    Eigen::Matrix4d rot;
    if (axis == 0) rot << 1, 0, 0, 0,
            0, std::cos(rads), std::sin(rads), 0,
            0, -std::sin(rads), std::cos(rads), 0,
            0, 0, 0, 1;
    else if (axis == 1)
        rot << std::cos(rads), 0, -std::sin(rads), 0,
            0, 1, 0, 0,
            std::sin(rads), 0, std::cos(rads), 0,
            0, 0, 0, 1;
    else rot << std::cos(rads), -std::sin(rads), 0, 0,
            std::sin(rads), std::cos(rads), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    translate(point, -origin);
    point = rot * point;
    translate(point, origin);
}

void getCube(std::vector<Eigen::Vector4d>& vertices, std::vector<Eigen::Vector3i>& indices) {
    double size = 0.5;
    double z = -1.5;
    vertices.push_back({-size, -size, -size + z, 1.0});
    vertices.push_back({-size, -size, size + z, 1.0});
    vertices.push_back({-size, size, -size + z, 1.0});
    vertices.push_back({-size, size, size + z, 1.0});
    vertices.push_back({size, -size, -size + z, 1.0});
    vertices.push_back({size, -size, size + z, 1.0});
    vertices.push_back({size, size, -size + z, 1.0});
    vertices.push_back({size, size, size + z, 1.0});

    indices.push_back({0, 1, 2});
    indices.push_back({1, 2, 3});
    indices.push_back({2, 3, 6});
    indices.push_back({3, 6, 7});
    indices.push_back({5, 6, 7});
    indices.push_back({4, 5, 6});
    indices.push_back({0, 1, 4});
    indices.push_back({1, 4, 5});
    indices.push_back({0, 2, 4});
    indices.push_back({2, 4, 6});
    indices.push_back({1, 3, 5});
    indices.push_back({3, 5, 7});
}