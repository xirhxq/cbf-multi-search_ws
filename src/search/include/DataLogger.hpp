/* 
  --- Requirements ---
    need add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/data\") in CMakeFiles.txt

  --- Example ---
    #include "DataLogger.hpp"

    int main() {
        DataLogger logger("xxx.csv");
        std::vector<std::string> variableNames = {"Var 1", "Var 2"};
        logger.initialize(variableNames);
    
        for (int i = 0;; ++i) {
            int var1 = i * 3;
            double var2 = 3.1415926 * i;

            logger.log("Var 2", var2);
            logger.log("Var 1", var1);
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
#include "Eigen/Core"

template<typename T>
struct is_eigen_type : std::false_type {};

template<typename T>
struct is_eigen_type<Eigen::MatrixBase<Derived> > : std::true_type {};

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
    void log(const std::string &variableName, const T &value) {
        if (file_.is_open() && initialized_) {
            if (is_eigen_type<T>::value) {
                parseEigenMatrixValue(variableName, value);
            }
            else if (std::is_same<T, int>::value || std::is_same<T, double>::value || std::is_same<T, float>::value){
                parseVariableValue(variableName, value);
            }
            else {
                parsePointValue(variableName, value);
            }
        }
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

    int findIndex(const std::string &variableName){
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
        } else if (variableType == "EigenMatrix") {
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
        ss << std::setw(6) << value;
        return ss.str();
    }

    void updateValue(const int& ind, const std::string &varValueStr) {
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