#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include "Eigen/Core"

struct Point {
    double x;
    double y;
    double z;
};

class DataLogger {
private:
    std::ofstream fileStream_;
    std::vector<std::string> variableNames_;
    std::vector<std::string> values_;
    std::vector<bool> valueUpdated_;

public:
    DataLogger(const std::string &filename) {
        fileStream_.open(filename);
    }

    void initialize(const std::vector<std::pair<std::string, std::string>> &variableInfo) {
        for (const auto &info: variableInfo) {
            const std::string &variableName = info.first;
            const std::string &variableType = info.second;
            parseVariableName(variableName, variableType);
        }
        writeHeader();
    }

    template<typename T>
    void log(const std::string &variableName, const T &value) {
        parseVariableValue(variableName, value);
    }

    void newline() {
        // Check if any value is not updated
        for (bool updated: valueUpdated_) {
            if (!updated) {
                std::cerr << "Error: Value not updated" << std::endl;
                return;
            }
        }

        // Write the values to the file
        for (const std::string &value: values_) {
            fileStream_ << value << ",";
        }
        fileStream_ << std::endl;

        // Reset the updated flags and clear the values
        valueUpdated_.assign(valueUpdated_.size(), false);
        values_.clear();
    }

    ~DataLogger() {
        fileStream_.close();
    }

private:
    void parseVariableName(const std::string &variableName, const std::string &variableType) {
        if (variableType == "int" || variableType == "double") {
            variableNames_.push_back(variableName);
        } else if (variableType == "EigenMatrix") {
            std::string name = variableName.substr(0, variableName.find_first_of('['));
            printf("variableName: %s\n", variableName.c_str());
            std::string rows = variableName.substr(
                    variableName.find_first_of('[') + 1,
                    variableName.find_first_of(']') - variableName.find_first_of('[') - 1
            );
            std::string cols = variableName.substr(
                    variableName.find_last_of('[') + 1,
                    variableName.find_last_of(']') - variableName.find_last_of('[') - 1
            );
            printf("rows: %s, cols: %s\n", rows.c_str(), cols.c_str());
            int numRows = std::stoi(rows);
            int numCols = std::stoi(cols);

            // Generate the matrix variable names
            for (int i = 0; i < numRows; ++i) {
                for (int j = 0; j < numCols; ++j) {
                    std::string varName = name + "[" + std::to_string(i) + "][" + std::to_string(j) + "]";
                    variableNames_.push_back(varName);
                }
            }
        } else if (variableType == "Point") {
            variableNames_.push_back(variableName + ".x");
            variableNames_.push_back(variableName + ".y");
            variableNames_.push_back(variableName + ".z");
        }
    }

    template<typename T>
    void parseVariableValue(const std::string &variableName, const T &value) {
        std::stringstream ss;
        ss << value;
        values_.push_back(ss.str());
        valueUpdated_.push_back(true);
    }

    void parseVariableValue(const std::string &variableName, const Eigen::MatrixXd &matrix) {
        for (int i = 0; i < matrix.rows(); ++i) {
            for (int j = 0; j < matrix.cols(); ++j) {
                std::stringstream ss;
                ss << matrix(i, j);
                values_.push_back(ss.str());
                valueUpdated_.push_back(true);
            }
        }
    }

    void parseVariableValue(const std::string &variableName, const Point &point) {
        values_.push_back(std::to_string(point.x));
        values_.push_back(std::to_string(point.y));
        values_.push_back(std::to_string(point.z));
        valueUpdated_.insert(valueUpdated_.end(), 3, true);
    }

    void writeHeader() {
        for (const std::string &variableName: variableNames_) {
            fileStream_ << variableName << ",";
        }
        fileStream_ << std::endl;
    }
};

int main() {
    DataLogger logger("data.csv");

    std::vector<std::pair<std::string, std::string>> variableInfo;
    variableInfo.emplace_back("var1", "int");
    variableInfo.emplace_back("var2", "double");
    variableInfo.emplace_back("matrix[3][2]", "EigenMatrix");
    variableInfo.emplace_back("point", "Point");

    logger.initialize(variableInfo);

    int var1 = 0;
    double var2 = 3.14;
    Eigen::Matrix2d matrix;
    matrix << 1, 2, 3, 4;
    Point point;
    point.x = 1.0;
    point.y = 2.0;
    point.z = 3.0;

    logger.log("var1", var1);
    logger.log("var2", var2);
    logger.newline();

    logger.log("matrix", matrix);
    logger.newline();

    logger.log("point", point);
    logger.newline();

    return 0;
}
