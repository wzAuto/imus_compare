#include <iostream>
#include <vector>
#include <fstream>
#include <Eigen/Dense>
#include "proto/fake_proto_serial.pb.h"

std::vector<pb_viewer::IMUAccGyro> loadImu(const std::string &path);
void dumpImu(const std::vector<pb_viewer::IMUAccGyro> &imus, const std::string &path);
uint8_t readAtHead(std::ifstream &ifs);
uint16_t readAtLength(std::ifstream &ifs);

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cout << "you may input a lsm6dsm log" << std::endl;
        return 0;
    }
    const std::string log_path = argv[1];
    auto imus = loadImu(log_path);
    std::cout << "imus: " << imus.size() << std::endl;
    dumpImu(imus, log_path + ".bin");
    return 0;
}

std::vector<pb_viewer::IMUAccGyro> loadImu(const std::string &path)
{
    std::vector<pb_viewer::IMUAccGyro> result;

    std::ifstream ifs(path);
    if (not ifs.is_open())
    {
        std::cout << "not open: " << path << std::endl;
        return {};
    }
    while (not ifs.eof())
    {
        if (readAtHead(ifs) != 0xEE)
        {
            continue;
        }
        auto len = readAtLength(ifs);
        std::vector<char> data;
        data.resize(len);
        ifs.read(data.data(), len);
        pb_viewer::Message message;
        if (not message.ParseFromArray(data.data(), len))
        {
            continue;
        }
        if (not message.has_imu_acc_gyro())
        {
            continue;
        }
        // if (fabsf(message.imu_acc_gyro().gyro_z()) > 1e1 / (4.375 * 1e-3 * M_PI / 180.0))
        // {
        //     continue;
        // }
        result.push_back(message.imu_acc_gyro());
    }
    ifs.close();
    return result;
}

void dumpImu(const std::vector<pb_viewer::IMUAccGyro> &imus, const std::string &path)
{
    const float acc_ratio = 0.061 * 1e-3 * 9.8;           // m/s2
    const float gyro_ratio = 4.375 * 1e-3 * M_PI / 180.0; // rad/s
    std::ofstream ofs(path);
    int idx = 0;
    for (const auto &imu : imus)
    {
        double tick_seconds = imu.tick() / 1000.0;
        float acc[3] = {imu.acc_x() * acc_ratio, imu.acc_y() * acc_ratio, imu.acc_z() * acc_ratio};
        float gyro[3] = {imu.gyro_x() * gyro_ratio, imu.gyro_y() * gyro_ratio, imu.gyro_z() * gyro_ratio};
        ofs.write((char *)(&tick_seconds), sizeof(double));
        ofs.write((char *)acc, 3 * sizeof(float));
        ofs.write((char *)gyro, 3 * sizeof(float));
        if (fabsf(gyro[2]) > 1)
        {
            std::cout << "big gyro: " << gyro[2] << " at: " << idx << " in " << imus.size() << std::endl;
        }
        ++idx;
    }
    ofs.close();
}

uint8_t readAtHead(std::ifstream &ifs)
{
    uint8_t data = 0;
    while (ifs.is_open() and (not ifs.eof()) and (data != 0xEE))
    {
        ifs.read((char *)&data, 1);
        // std::cout << "read head with: " << (data == 0xEE) << std::endl;
    }
    // std::cout << "read head with: " << (data == 0xEE) << std::endl;
    return data;
}

uint16_t readAtLength(std::ifstream &ifs)
{
    uint16_t length = 0;
    ifs.read((char *)(&length), sizeof(length));
    return length;
}