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
        std::cout << "you may input a mpu6050 log" << std::endl;
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
        std::vector<char> data;
        data.resize(18);
        ifs.read(data.data(), 18);
        for (int idx = 4; idx < 18; idx += 2)
        {
            std::swap(data[idx], data[idx + 1]);
        }
        pb_viewer::IMUAccGyro imu;
        imu.set_tick(*(uint32_t *)data.data());
        imu.set_acc_x(*(int16_t *)(data.data() + 4));
        imu.set_acc_y(*(int16_t *)(data.data() + 6));
        imu.set_acc_z(*(int16_t *)(data.data() + 8));

        imu.set_gyro_x(*(int16_t *)(data.data() + 12));
        imu.set_gyro_y(*(int16_t *)(data.data() + 14));
        imu.set_gyro_z(*(int16_t *)(data.data() + 16));

        result.push_back(imu);
    }
    ifs.close();
    return result;
}

void dumpImu(const std::vector<pb_viewer::IMUAccGyro> &imus, const std::string &path)
{
    const float acc_ratio = 1 / 16384.0f * 9.8;         // m/s2
    const float gyro_ratio = 1 / 131.0f * M_PI / 180.0; // rad/s
    std::ofstream ofs(path);
    for (const auto &imu : imus)
    {
        double tick_seconds = imu.tick() / 1000.0;
        float acc[3] = {imu.acc_x() * acc_ratio, imu.acc_y() * acc_ratio, imu.acc_z() * acc_ratio};
        float gyro[3] = {imu.gyro_x() * gyro_ratio, imu.gyro_y() * gyro_ratio, imu.gyro_z() * gyro_ratio};
        ofs.write((char *)(&tick_seconds), sizeof(double));
        ofs.write((char *)acc, 3 * sizeof(float));
        ofs.write((char *)gyro, 3 * sizeof(float));
        // std::cout << "imu: " << acc[2] << std::endl;
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