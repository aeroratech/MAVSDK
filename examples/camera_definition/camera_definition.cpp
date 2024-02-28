#include <iostream>
#include <thread>
#include <chrono>
#include <future>
#include <filesystem>
#include <fstream>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/camera_server/camera_server.h>
#include <mavsdk/plugins/camera/camera.h>
#include <mavsdk/plugins/ftp_server/ftp_server.h>
#include <mavsdk/plugins/ftp/ftp.h>
#include <mavsdk/plugins/param_server/param_server.h>

// manually set camrea defintion data
static void set_camera_defintion(std::shared_ptr<mavsdk::System> system, mavsdk::Camera& camera);
// mavlink ftp to download camera config file
static std::string download_camera_definition_file_by_ftp(
    std::shared_ptr<mavsdk::System> system, std::string file_uri);

int main(int argc, char** argv)
{
    // We run the server plugins on a seperate thread so we can use the main
    // thread as a ground station.
    std::thread autopilotThread([]() {
        mavsdk::Mavsdk mavsdk_camera{
            mavsdk::Mavsdk::Configuration{mavsdk::Mavsdk::ComponentType::Camera}};

        // 14030 is the default camera port for PX4 SITL
        auto result = mavsdk_camera.add_any_connection("udp://127.0.0.1:14030");
        if (result != mavsdk::ConnectionResult::Success) {
            std::cerr << "Could not establish connection: " << result << std::endl;
            return 1;
        }
        std::cout << "Created camera server connection" << std::endl;

        auto camera_server = mavsdk::CameraServer{mavsdk_camera.server_component()};
        auto param_server = mavsdk::ParamServer{mavsdk_camera.server_component()};

        param_server.subscribe_changed_param_float([](mavsdk::ParamServer::FloatParam float_param) {
            std::cout << "param server change " << float_param.name << " to " << float_param.value
                      << std::endl;
        });
        param_server.subscribe_changed_param_int([](mavsdk::ParamServer::IntParam int_param) {
            std::cout << "param server change " << int_param.name << " to " << int_param.value
                      << std::endl;
        });
        // demo for param server camera param
        param_server.provide_param_int("CAM_MODE", 1);
        param_server.provide_param_int("CAM_WBMODE", 0);
        param_server.provide_param_int("CAM_EXPMODE", 0);
        param_server.provide_param_float("CAM_EV", 0);
        param_server.provide_param_int("CAM_ISO", 100);
        param_server.provide_param_float("CAM_SHUTTERSPD", 0.01);
        param_server.provide_param_int("CAM_VIDFMT", 1);
        param_server.provide_param_int("CAM_VIDRES", 0);
        param_server.provide_param_int("CAM_PHOTORATIO", 1);

        auto ret = camera_server.set_information({
            .vendor_name = "MAVSDK",
            .model_name = "Example Camera Server",
            .firmware_version = "0.1.0.12",
            .focal_length_mm = 3.0,
            .horizontal_sensor_size_mm = 3.68,
            .vertical_sensor_size_mm = 2.76,
            .horizontal_resolution_px = 3280,
            .vertical_resolution_px = 2464,
            .lens_id = 0,
            .definition_file_version = 1,
            .definition_file_uri = "C10.xml",
            .camera_cap_flags = {},
        });

        auto ftp_server = mavsdk::FtpServer{mavsdk_camera.server_component()};
        ftp_server.set_root_dir(std::filesystem::current_path().append("definition"));
        while (true) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    });

    // we run client plugins to act as the GCS
    // to communicate with the camera server plugins.
    mavsdk::Mavsdk mavsdk{
        mavsdk::Mavsdk::Configuration{mavsdk::Mavsdk::ComponentType::GroundStation}};

    auto result = mavsdk.add_any_connection("udp://:14030");
    if (result == mavsdk::ConnectionResult::Success) {
        std::cout << "Connected!" << std::endl;
    }

    auto prom = std::promise<std::shared_ptr<mavsdk::System>>{};
    auto fut = prom.get_future();
    mavsdk::Mavsdk::NewSystemHandle handle =
        mavsdk.subscribe_on_new_system([&mavsdk, &prom, &handle]() {
            auto system = mavsdk.systems().back();

            if (system->has_camera()) {
                std::cout << "Discovered camera from Client" << std::endl;

                // Unsubscribe again as we only want to find one system.
                mavsdk.unsubscribe_on_new_system(handle);
                prom.set_value(system);
            } else {
                std::cout << "No MAVSDK found" << std::endl;
            }
        });

    if (fut.wait_for(std::chrono::seconds(10)) == std::future_status::timeout) {
        std::cout << "No camera found, exiting" << std::endl;
        return -1;
    }

    auto system = fut.get();
    auto camera = mavsdk::Camera{system};
    set_camera_defintion(system, camera);

    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}

static void set_camera_defintion(std::shared_ptr<mavsdk::System> system, mavsdk::Camera& camera)
{
    auto prom = std::promise<std::string>{};
    auto fut = prom.get_future();
    auto handle = camera.subscribe_information([&prom](mavsdk::Camera::Information info) {
        std::cout << info << std::endl;
        prom.set_value(info.definition_file_uri);
    });

    if (fut.wait_for(std::chrono::seconds(10)) == std::future_status::timeout) {
        std::cout << "No camera info get" << std::endl;
        return;
    }
    camera.unsubscribe_information(handle);
    std::string camera_definition_data = download_camera_definition_file_by_ftp(system, fut.get());
    auto result = camera.set_definition_data(camera_definition_data);
    std::cout << "Set camera definition result : " << result << std::endl;

    // demo for set camera definition value
    mavsdk::Camera::Setting setting;
    setting.setting_id = "CAM_EV";
    setting.option.option_id = "2.0";
    auto set_result = camera.set_setting(setting);
    std::cout << "set " << setting.setting_id << " value : " << setting.option.option_id
              << " result : " << set_result << std::endl;
    setting.option.option_id = "-1";
    auto get_result = camera.get_setting(setting);
    std::cout << "get " << get_result.second.setting_id
              << " value : " << get_result.second.option.option_id
              << " result : " << get_result.first << std::endl;
}

static std::string
download_camera_definition_file_by_ftp(std::shared_ptr<mavsdk::System> system, std::string file_uri)
{
    auto ftp = mavsdk::Ftp{system};
    std::filesystem::path download_path = std::filesystem::current_path().append("build");

    auto prom = std::promise<std::string>{};
    auto fut = prom.get_future();
    ftp.download_async(
        file_uri,
        download_path.string(),
        false,
        [&download_path, &file_uri, &prom](
            mavsdk::Ftp::Result result, mavsdk::Ftp::ProgressData progress_data) {
            if (result == mavsdk::Ftp::Result::Success) {
                std::string file_path_with_name = download_path;
                file_path_with_name += "/" + file_uri;
                std::cout << "Download complete ,output path :" << file_path_with_name << std::endl;
                std::ifstream file_stream(file_path_with_name);
                std::stringstream buffer;
                buffer << file_stream.rdbuf();
                prom.set_value(buffer.str());
            } else if (result == mavsdk::Ftp::Result::Next) {
                std::cout << "Download progress: " << progress_data.bytes_transferred << "/"
                          << progress_data.total_bytes << " bytes" << std::endl;
            } else {
                std::cout << "Download result : " << result << std::endl;
            }
        });

    if (fut.wait_for(std::chrono::seconds(10)) == std::future_status::timeout) {
        std::cout << "Download camera define file failed" << std::endl;
        return "";
    }

    return fut.get();
}