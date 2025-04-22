#include <igl/readSTL.h>
#include <igl/cotmatrix.h>
#include <igl/massmatrix.h>
#include <igl/slice.h>
#include <Eigen/Dense>
#include <fstream>
#include <vector>
#include <iostream>

int main()
{
    // 1. surface mesh
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    Eigen::MatrixXd N;
    igl::readSTL("/home/nrs/catkin_ws/src/nrs_path/mesh/workpiece.stl", V, F, N);

    // 2. projected path 불러오기
    Eigen::MatrixXd P;
    std::ifstream file("/home/nrs/catkin_ws/src/nrs_path/data/Ori_path_projected.txt");
    std::vector<Eigen::RowVector3d> path;
    double x, y, z;
    while (file >> x >> y >> z)
    {
        path.emplace_back(x, y, z);
    }
    P.resize(path.size(), 3);
    for (size_t i = 0; i < path.size(); ++i)
        P.row(i) = path[i];

    // 3. smoothing 준비
    // Cotangent Laplacian L, mass matrix M
    Eigen::SparseMatrix<double> L, M;
    igl::cotmatrix(V, F, L);
    igl::massmatrix(V, F, igl::MASSMATRIX_TYPE_VORONOI, M);

    // 4. 경로 smoothing (Lx = Mx 방정식 풀기)
    Eigen::MatrixXd P_smooth = P;
    Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> solver;
    solver.compute(M - 0.01 * L); // lambda=0.01 정도
    if (solver.info() != Eigen::Success)
    {
        std::cerr << "Solver failed!" << std::endl;
        return -1;
    }
    for (int dim = 0; dim < 3; ++dim)
    {
        Eigen::VectorXd b = M * P.col(dim);
        P_smooth.col(dim) = solver.solve(b);
    }

    // 5. 결과 저장
    std::ofstream out("/home/nrs/catkin_ws/src/nrs_path/data/Projected_Path_Smooth.txt");
    for (int i = 0; i < P_smooth.rows(); ++i)
        out << P_smooth(i, 0) << " " << P_smooth(i, 1) << " " << P_smooth(i, 2) << "\n";

    std::cout << "[✔] Smoothed path saved to Projected_Path_Smooth.txt\n";
    return 0;
}
