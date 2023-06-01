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
        variableInfo.emplace_back("var1", "int");
        variableInfo.emplace_back("var2", "double");
        variableInfo.emplace_back("a[3][2]", "EigenMatrix");
        variableInfo.emplace_back("b[3][1]", "EigenVector");
        variableInfo.emplace_back("c[" + std::to_string(c.rows()) + "][" + std::to_string(c.cols()) + "]", "EigenVector");
        variableInfo.emplace_back("point", "Point");
        variableInfo.emplace_back("anotherPoint", "Point");

        logger.initialize(variableInfo);

        for (int i = 0; i < 10; ++i) {
            int var1 = i;
            double var2 = 3.14 * i;
            Point point{1.0 * i, 2.0 * i, 3.0 * i};
            AnotherPoint anotherPoint{-1.0 * i, -2.0 * i, -3.0 * i};
            a << 1 * i, 2 * i, 3 * i, 4 * i, 5 * i, 6 * i;
            b << 10 * i, 20 * i, 30 * i;
            c << 100 * i, 200 * i, 300 * i;

            logger.log("var1", var1);
            logger.log("var2", var2);

            // can log Eigen::Matrix, Eigen::Vector for any size, any type(double, int, float)
            logger.log("a", a);
            logger.log("b", b);
            logger.log("c", c);

            // log function can be used not by the order of variableInfo
            logger.log("anotherPoint", anotherPoint);
            logger.log("point", point);
            logger.newline();
        }
    }

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
            filename_(std::string(ROOT_DIR) + getCurrentTimestamp('-') + "_" + filename),
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
    typename std::enable_if<!is_eigen_matrix<T>::value &&
                            std::is_arithmetic<T>::value>::type
    log(const std::string &variableName, const T &value) {
        parseVariableValue(variableName, value);
    }

    template<typename T>
    typename std::enable_if<!is_eigen_matrix<T>::value &&
                            !std::is_arithmetic<T>::value>::type
    log(const std::string &variableName, const T &value) {
        parsePointValue(variableName, value);
    }


    void newline() {
        if (file_.is_open() && initialized_) {
            for (const auto &updated: valueUpdated_) {
                if (!updated) {
                    std::cerr << "Some value has not been updated!!!" << std::endl;
                    break;
                }
            }

            file_ << getCurrentTimestamp();
            for (size_t i = 0; i < values_.size(); ++i) {
                file_ << "," << values_[i];
                valueUpdated_[i] = false;
            }
            file_ << std::endl;

        }
    }

    std::string getCurrentTimestamp(char sep = ',') {
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
        } else if (variableType == "EigenMatrix" || variableType == "EigenVector") {
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
        } else if (variableType == "Point" || variableType == "point") {
            nameTypeMap[variableName] = variableType;
            variableNames_.push_back(variableName + ".x");
            variableNames_.push_back(variableName + ".y");
            variableNames_.push_back(variableName + ".z");
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

    void writeHeader() {
        file_ << "Year, Month, Day, Hour, Minute, Second, Millisecond";
        for (const auto &name: variableNames_) {
            file_ << ", " << name;
        }
        file_ << std::endl;
    }
};

#endif
