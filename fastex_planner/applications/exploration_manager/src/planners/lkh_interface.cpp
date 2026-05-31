
#include <fstream>
#include <sstream>

#include "exploration_manager/planners/lkh_interface.h"

namespace fastex_explorer
{
LkhInterface::LkhInterface() { solver_ = std::make_unique<lkh_solver::LKHSlover>(); }

bool LkhInterface::solveATSP(const std::string& file_name, const Eigen::MatrixXd& matrix,
                             std::vector<int>& optimal_indices)
{
    writeATSPFiles(file_name, matrix);
    if (!solver_->solveProblem(file_name + ".par"))
        return false;

    return readOptimalTour(file_name, optimal_indices);
}

bool LkhInterface::solveSOP(const std::string& file_name, const Eigen::MatrixXd& matrix,
                            std::vector<int>& optimal_indices)
{
    writeSOPFiles(file_name, matrix);
    if (!solver_->solveProblem(file_name + ".par"))
        return false;

    return readOptimalTour(file_name, optimal_indices);
}

void LkhInterface::writeATSPFiles(const std::string& file_name, const Eigen::MatrixXd& matrix) const
{
    const int dimension = matrix.rows();

    std::ofstream par_file(file_name + ".par");
    par_file << "SPECIAL\n";
    par_file << "PROBLEM_FILE = " << file_name + ".tsp\n";
    par_file << "GAIN23 = NO\n";
    par_file << "OUTPUT_TOUR_FILE =" << file_name + ".tour\n";
    par_file << "RUNS = 1\n";

    std::ofstream prob_file(file_name + ".tsp");
    prob_file << "NAME : single\n";
    prob_file << "TYPE : ATSP\n";
    prob_file << "DIMENSION : " << dimension << "\n";
    prob_file << "EDGE_WEIGHT_TYPE : EXPLICIT\n";
    prob_file << "EDGE_WEIGHT_FORMAT : FULL_MATRIX\n";
    prob_file << "EDGE_WEIGHT_SECTION\n";

    const int scale = 100;
    for (int i = 0; i < dimension; ++i)
    {
        for (int j = 0; j < dimension; ++j)
            prob_file << static_cast<int>(matrix(i, j) * scale) << " ";

        prob_file << "\n";
    }

    prob_file << "EOF";
}

void LkhInterface::writeSOPFiles(const std::string& file_name, const Eigen::MatrixXd& matrix) const
{
    const int dimension = matrix.rows();

    std::ofstream par_file(file_name + ".par");
    par_file << "SPECIAL\n";
    par_file << "RUNS = 1\n";
    par_file << "PROBLEM_FILE = " << file_name + ".sop\n";
    par_file << "TOUR_FILE =" << file_name + ".tour\n";

    std::ofstream prob_file(file_name + ".sop");
    prob_file << "NAME: Single Local SOP\n";
    prob_file << "TYPE: SOP\n";
    prob_file << "DIMENSION: " << dimension << "\n";
    prob_file << "EDGE_WEIGHT_TYPE: EXPLICIT\n";
    prob_file << "EDGE_WEIGHT_FORMAT: FULL_MATRIX\n";
    prob_file << "EDGE_WEIGHT_SECTION\n";
    prob_file << dimension << "\n";

    const int scale = 100;
    std::ostringstream prob_data_stream;
    for (int i = 0; i < dimension; ++i)
    {
        for (int j = 0; j < dimension; ++j)
        {
            if (matrix(i, j) > 0)
                prob_data_stream << static_cast<int>(matrix(i, j) * scale) << " ";
            else
                prob_data_stream << "-1 ";
        }
        prob_data_stream << "\n";
    }

    prob_file << prob_data_stream.str();
    prob_file << "EOF";
}

bool LkhInterface::readOptimalTour(const std::string& file_name,
                                   std::vector<int>& optimal_indices) const
{
    optimal_indices.clear();

    std::ifstream res_file(file_name + ".tour");
    if (!res_file.is_open())
        return false;

    std::string line;
    while (std::getline(res_file, line))
    {
        if (line == "TOUR_SECTION")
            break;
    }

    int idx;
    while (res_file >> idx)
    {
        if (idx == -1)
            break;

        optimal_indices.push_back(idx);
    }

    return !optimal_indices.empty();
}
} // namespace fastex_explorer
