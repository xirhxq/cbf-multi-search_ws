/*
  --- Requirements ---
    need add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/data\") in CMakeFiles.txt

  --- Example ---
    #include "DataLogger.hpp"
    #include "Eigen/Core"

    struct Point {
        double x;
        double y;
        double z;
    };

    struct AnotherPoint {
        double x;
        double y;
        double z;
    };

    int main() {
        Eigen::MatrixXd a(3, 2);
        Eigen::VectorXd b(3);
        Eigen::VectorXi c(3);

        DataLogger logger("data.csv");

        std::vector<std::pair<std::string, std::string>> variableInfo;

        // for primitive type
        variableInfo.emplace_back("var1", "int");
        variableInfo.emplace_back("var2", "double");

        // for eigen matrix or vector you should specify the size
        variableInfo.emplace_back("a[3][2]", "EigenMatrix");
        variableInfo.emplace_back("b[3][1]", "EigenVector");

        // also recommend to use .rows() and .cols()
        variableInfo.emplace_back("c[" + std::to_string(c.rows()) + "][" + std::to_string(c.cols()) + "]", "EigenVector");

        // for any type with .x .y .z members
        variableInfo.emplace_back("point", "Point");
        variableInfo.emplace_back("anotherPoint", "Point");

        // for array you should specify the size
        variableInfo.emplace_back("array[3]", "array");

        logger.initialize(variableInfo);

        for (int i = 0; i < 10; ++i) {
            int var1 = i;
            double var2 = 3.1415926 * i;
            Point point{1.0 * i, 2.0 * i, 3.0 * i};
            AnotherPoint anotherPoint{-1.0 * i, -2.0 * i, -3.0 * i};
            a << 1 * i, 2 * i, 3 * i, 4 * i, 5 * i, 6 * i;
            b << 10 * i, 20 * i, 30 * i;
            c << 100 * i, 200 * i, 300 * i;
            int array[3] = {1 * i, 2 * i, 3 * i};

            logger.log("var1", var1);
            logger.log("var2", var2);

            // can log Eigen::Matrix, Eigen::Vector for any size, any type(double, int, float)
            logger.log("a", a);
            logger.log("b", b);
            logger.log("c", c);
            logger.log("array", array);

            // log function can be used not by the order of variableInfo
            logger.log("anotherPoint", anotherPoint);
            logger.log("point", point);
            logger.newline();
        }
    }

    --- Output (data/2023-06-02-01-27-49-890_data.csv) ---
    Year, Month, Day, Hour, Minute, Second, Millisecond, var1, var2, a[0][0], a[0][1], a[1][0], a[1][1], a[2][0], a[2][1], b[0][0], b[1][0], b[2][0], c[0][0], c[1][0], c[2][0], point.x, point.y, point.z, anotherPoint.x, anotherPoint.y, anotherPoint.z, array[0], array[1], array[2]
    2023, 06, 02, 01, 27, 49, 891, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -0, -0, -0, 0, 0, 0
    2023, 06, 02, 01, 27, 49, 891, 1, 3.14159, 1, 2, 3, 4, 5, 6, 10, 20, 30, 100, 200, 300, 1, 2, 3, -1, -2, -3, 1, 2, 3
    2023, 06, 02, 01, 27, 49, 891, 2, 6.28319, 2, 4, 6, 8, 10, 12, 20, 40, 60, 200, 400, 600, 2, 4, 6, -2, -4, -6, 2, 4, 6
    2023, 06, 02, 01, 27, 49, 891, 3, 9.42478, 3, 6, 9, 12, 15, 18, 30, 60, 90, 300, 600, 900, 3, 6, 9, -3, -6, -9, 3, 6, 9
    2023, 06, 02, 01, 27, 49, 891, 4, 12.5664, 4, 8, 12, 16, 20, 24, 40, 80, 120, 400, 800, 1200, 4, 8, 12, -4, -8, -12, 4, 8, 12
    2023, 06, 02, 01, 27, 49, 891, 5, 15.708, 5, 10, 15, 20, 25, 30, 50, 100, 150, 500, 1000, 1500, 5, 10, 15, -5, -10, -15, 5, 10, 15
    2023, 06, 02, 01, 27, 49, 891, 6, 18.8496, 6, 12, 18, 24, 30, 36, 60, 120, 180, 600, 1200, 1800, 6, 12, 18, -6, -12, -18, 6, 12, 18
    2023, 06, 02, 01, 27, 49, 891, 7, 21.9911, 7, 14, 21, 28, 35, 42, 70, 140, 210, 700, 1400, 2100, 7, 14, 21, -7, -14, -21, 7, 14, 21
    2023, 06, 02, 01, 27, 49, 891, 8, 25.1327, 8, 16, 24, 32, 40, 48, 80, 160, 240, 800, 1600, 2400, 8, 16, 24, -8, -16, -24, 8, 16, 24
    2023, 06, 02, 01, 27, 49, 891, 9, 28.2743, 9, 18, 27, 36, 45, 54, 90, 180, 270, 900, 1800, 2700, 9, 18, 27, -9, -18, -27, 9, 18, 27

*/

#ifndef DATALOGGER
#define DATALOGGER

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <ctime>
#include <chrono>
#include <iomanip>
#include <thread>
#include <sstream>
#include <type_traits>
#include <map>
#include "Eigen/Core"

template <typename T>
struct is_eigen_matrix : std::false_type {};

template <typename T, int R, int C>
struct is_eigen_matrix<Eigen::Matrix<T, R, C>> : std::true_type {};

class DataLogger {
public:
    DataLogger(const std::string &filename) :
            filename_(std::string(ROOT_DIR) + getCurrentTimestamp("-") + "_" + filename),
            initialized_(false) {
        file_.open(filename_);
        while (!file_.is_open()) {
            std::cerr << "Couldn't open file: " << filename_ << std::endl;
        }
    }

    ~DataLogger() {
        if (file_.is_open()) {
            file_.close();
        }
    }

    void initialize(const std::vector<std::pair<std::string, std::string>> &variableInfo) {
        if (!initialized_) {
            initialized_ = true;
            for (const auto &info: variableInfo) {
                const std::string &variableName = info.first;
                const std::string &variableType = info.second;
                parseVariableName(variableName, variableType);
            }
            writeHeader();

            valueUpdated_.resize(variableNames_.size(), false);
            values_.resize(variableNames_.size());
        }
    }

    template<typename T>
    typename std::enable_if<is_eigen_matrix<T>::value>::type
    log(const std::string &variableName, const T &value) {
        parseEigenMatrixValue(variableName, value);
    }

    template<typename T>
    typename std::enable_if<std::is_arithmetic<T>::value>::type
    log(const std::string &variableName, const T &value) {
        parseVariableValue(variableName, value);
    }

    template<typename T>
    typename std::enable_if<!is_eigen_matrix<T>::value &&
                            !std::is_arithmetic<T>::value &&
                            !std::is_array<T>::value>::type
    log(const std::string &variableName, const T &value) {
        parsePointValue(variableName, value);
    }

    template<typename T>
    typename std::enable_if<std::is_array<T>::value>::type
    log(const std::string &variableName, const T &value) {
        parseArrayValue(variableName, value);
    }

    void newline() {
        if (file_.is_open() && initialized_) {
            for (const auto &updated: valueUpdated_) {
                if (!updated) {
                    std::cerr << "Some value has not been updated!!!" << std::endl;
                    break;
                }
            }

            file_ << getCurrentTimestamp(", ");
            for (size_t i = 0; i < values_.size(); ++i) {
                file_ << ", " << values_[i];
                valueUpdated_[i] = false;
            }
            file_ << std::endl;

        }
    }

    std::string getCurrentTimestamp(std::string sep=",") {
        auto now = std::chrono::system_clock::now();
        auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
        auto now_time = std::chrono::system_clock::to_time_t(now);
        auto now_tm = std::localtime(&now_time);

        std::stringstream ss;
        ss << now_tm->tm_year + 1900 << sep
           << std::setfill('0') << std::setw(2) << now_tm->tm_mon + 1 << sep
           << std::setfill('0') << std::setw(2) << now_tm->tm_mday << sep
           << std::setfill('0') << std::setw(2) << now_tm->tm_hour << sep
           << std::setfill('0') << std::setw(2) << now_tm->tm_min << sep
           << std::setfill('0') << std::setw(2) << now_tm->tm_sec << sep
           << std::setfill('0') << std::setw(3) << now_ms.time_since_epoch().count() % 1000;

        return ss.str();
    }

private:
    std::string filename_;
    std::ofstream file_;
    bool initialized_;
    std::vector<std::string> variableNames_;
    std::vector<std::string> values_;
    std::vector<bool> valueUpdated_;
    std::map<std::string, std::string> nameTypeMap;
    std::map<std::string, int> arrayNameSizeMap;

    int findIndex(const std::string &variableName) {
        auto it = std::find(variableNames_.begin(), variableNames_.end(), variableName);
        if (it != variableNames_.end()) {
            size_t index = std::distance(variableNames_.begin(), it);
            return index;
        } else {
            std::cerr << "Couldn't find var: " << variableName << std::endl;
            return -1;
        }
    }

    void parseVariableName(const std::string &variableName, const std::string &variableType) {
        if (variableType == "int" || variableType == "double") {
            nameTypeMap[variableName] = variableType;
            variableNames_.push_back(variableName);
        } else if (variableType.find("Eigen") != std::string::npos ||
                   variableType.find("eigen") != std::string::npos) {
            std::string name = variableName.substr(0, variableName.find_first_of('['));
            nameTypeMap[name] = variableType;
            std::string rows = variableName.substr(
                    variableName.find_first_of('[') + 1,
                    variableName.find_first_of(']') - variableName.find_first_of('[') - 1
            );
            std::string cols = variableName.substr(
                    variableName.find_last_of('[') + 1,
                    variableName.find_last_of(']') - variableName.find_last_of('[') - 1
            );
            int numRows = std::stoi(rows);
            int numCols = std::stoi(cols);

            for (int i = 0; i < numRows; ++i) {
                for (int j = 0; j < numCols; ++j) {
                    std::string varName = name + "[" + std::to_string(i) + "][" + std::to_string(j) + "]";
                    variableNames_.push_back(varName);
                }
            }
        } else if (variableType.find("point") != std::string::npos ||
                   variableType.find("Point") != std::string::npos) {
            nameTypeMap[variableName] = variableType;
            variableNames_.push_back(variableName + ".x");
            variableNames_.push_back(variableName + ".y");
            variableNames_.push_back(variableName + ".z");
        }
        else if (variableType.find("array") != std::string::npos ||
                 variableType.find("Array") != std::string::npos) {
            nameTypeMap[variableName] = variableType;
            std::string name = variableName.substr(0, variableName.find_first_of('['));
            std::string size = variableName.substr(
                    variableName.find_first_of('[') + 1,
                    variableName.find_first_of(']') - variableName.find_first_of('[') - 1
            );
            int numElements = std::stoi(size);
            arrayNameSizeMap[name] = numElements;
            for (int i = 0; i < numElements; ++i) {
                std::string varName = name + "[" + std::to_string(i) + "]";
                variableNames_.push_back(varName);
            }
        } else {
            std::cerr << "Unknown variable type: " << variableType << std::endl;
        }
    }

    template<typename T>
    std::string getValStr(const T &value) {
        std::stringstream ss;
        ss << value;
        return ss.str();
    }

    void updateValue(const int &ind, const std::string &varValueStr) {
        values_[ind] = varValueStr;
        valueUpdated_[ind] = true;
    }

    template<typename T>
    void parsePointValue(const std::string &variableName, const T &value) {
        updateValue(findIndex(variableName + ".x"), getValStr(value.x));
        updateValue(findIndex(variableName + ".y"), getValStr(value.y));
        updateValue(findIndex(variableName + ".z"), getValStr(value.z));
    }

    template<typename T>
    void parseVariableValue(const std::string &variableName, const T &value) {
        int index = findIndex(variableName);
        updateValue(index, getValStr(value));
    }

    template<typename T>
    void parseEigenMatrixValue(const std::string &variableName, const T &matrix) {
        for (int i = 0; i < matrix.rows(); ++i) {
            for (int j = 0; j < matrix.cols(); ++j) {
                int index = findIndex(
                        variableName + "[" + std::to_string(i) + "][" + std::to_string(j) + "]"
                );
                updateValue(index, getValStr(matrix(i, j)));
            }
        }
    }

    template<typename T>
    void parseArrayValue(const std::string &variableName, const T &array) {
        for (int i = 0; i < arrayNameSizeMap[variableName]; ++i) {
            int index = findIndex(variableName + "[" + std::to_string(i) + "]");
            updateValue(index, getValStr(array[i]));
        }
    }

    void writeHeader() {
        file_ << "Year, Month, Day, Hour, Minute, Second, Millisecond";
        for (const auto &name: variableNames_) {
            file_ << ", " << name;
        }
        file_ << std::endl;
    }
};

#endif
