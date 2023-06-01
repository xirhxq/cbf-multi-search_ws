#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <ctime>
#include <chrono>
#include <iomanip>
#include <thread>
#include <sstream>

class DataLogger {
public:
    // need add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\") in CMakeFiles.txt
    DataLogger(const std::string &filename) :
            filename_(std::string(ROOT_DIR) + getCurrentTimestamp('-') + "_" + filename),
            initialized_(false) {
        file_.open(filename_);
        while (!file_.is_open()) {
            std::cerr << "Couldn't open file：" << filename_ << std::endl;
        }
    }

    ~DataLogger() {
        if (file_.is_open()) {
            file_.close();
        }
    }

    void initialize(const std::vector<std::string> &variableNames) {
        if (!initialized_) {
            initialized_ = true;
            variableNames_ = variableNames;

            file_ << "Year, Month, Day, Hour, Minute, Second, Millisecond";
            for (const auto &name: variableNames_) {
                file_ << ", " << name;
            }
            file_ << std::endl;

            valueUpdated_.resize(variableNames_.size(), false);
            values_.resize(variableNames_.size());
        }
    }

    template<typename T>
    void log(const std::string &variableName, const T &value) {
        if (file_.is_open() && initialized_) {
            auto it = std::find(variableNames_.begin(), variableNames_.end(), variableName);
            if (it != variableNames_.end()) {
                size_t index = std::distance(variableNames_.begin(), it);

                std::stringstream ss;
                ss << std::setw(6) << value;
                values_[index] = ss.str();
                valueUpdated_[index] = true;
            } else {
                std::cerr << "Couldn't find var：" << variableName << std::endl;
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
};

int main() {
    DataLogger logger("data_log.csv");

    std::vector<std::string> variableNames = {"Variable 1", "Variable 2"};
    logger.initialize(variableNames);


    for (int i = 0; i < 2000; ++i) {
        int var1 = i * 3;
        double var2 = 3.1415926 * i;

        logger.log("Variable 2", var2);
        logger.log("Variable 1", var1);
        logger.newline();

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    return 0;
}
