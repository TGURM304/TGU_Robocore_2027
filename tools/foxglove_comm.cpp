//
// Created by Interweave on 2026/4/14.
//

#include "foxglove_comm.hpp"

#include <cmath>
#include <cstddef>
#include <cstring>
#include <locale>
#include <optional>
#include <sstream>
#include <unordered_map>
#include <utility>
#include <vector>

#include "foxglove/server.hpp"
#include "foxglove/schemas.hpp"
#include "foxglove/channel.hpp"
#include "foxglove/messages.hpp"
#include "tools/logger.hpp"


namespace tools {
    namespace {
        constexpr char FLOAT_JSON_SCHEMA[] = R"({
            "type": "object",
            "properties": {
                "value": {
                    "type": "number"
                }
            },
            "required": ["value"]
        })";

        constexpr uint32_t POINT_CLOUD_FLOAT32_TYPE = 7;
        constexpr uint32_t POINT_CLOUD_FIELD_COUNT = 3;
        constexpr uint32_t POINT_CLOUD_POINT_STRIDE = POINT_CLOUD_FIELD_COUNT * sizeof(float);

        foxglove::messages::Timestamp make_timestamp(uint64_t timestamp_ns) {
            foxglove::messages::Timestamp stamp;
            stamp.sec = static_cast<decltype(stamp.sec)>(timestamp_ns / 1000000000ULL);
            stamp.nsec = static_cast<decltype(stamp.nsec)>(timestamp_ns % 1000000000ULL);
            return stamp;
        }

        foxglove::messages::PackedElementField make_float32_field(const std::string &name, uint32_t offset) {
            foxglove::messages::PackedElementField field;
            field.name = name;
            field.offset = offset;
            field.type = static_cast<decltype(field.type)>(POINT_CLOUD_FLOAT32_TYPE);
            return field;
        }
    }

    struct FoxGloveComm::Impl {
        std::string host;
        uint16_t port = 0;
        bool ready = false;

        std::optional<foxglove::WebSocketServer> server;

        LoggerConfig cfg{.level = LogLevel::Debug, .enable_console = true, .enable_file = false};

        std::unordered_map<std::string, foxglove::messages::CompressedImageChannel> image_channels;

        std::unordered_map<std::string, foxglove::RawChannel> float_channels;

        std::unordered_map<std::string, foxglove::messages::PointCloudChannel> point_cloud_channels;

        Impl(const std::string &h, uint16_t p) : host(h), port(p) {
        }

        bool create_server() {
            ready = false;
            server.reset();

            auto result = foxglove::WebSocketServer::create(
                foxglove::WebSocketServerOptions{.host = host, .port = port});

            if (!result.has_value()) {
                LOG_ERROR(MODULE, "Failed to create WebSocket server at {}:{}", host, port);
                return false;
            }

            server.emplace(std::move(result.value()));
            ready = true;

            LOG_INFO(MODULE, "Foxglove server created at {}:{}", host, port);
            return true;
        }
    };

    FoxGloveComm::FoxGloveComm(const std::string &host, uint16_t port)
        : impl_(std::make_unique<Impl>(host, port)) {
        Logger::instance().init(impl_->cfg);
        impl_->create_server();
    }

    FoxGloveComm::~FoxGloveComm() = default;

    FoxGloveComm::FoxGloveComm(FoxGloveComm &&) noexcept = default;

    FoxGloveComm &FoxGloveComm::operator=(FoxGloveComm &&) noexcept = default;

    bool FoxGloveComm::is_ok() const {
        return impl_ && impl_->ready;
    }

    const std::string &FoxGloveComm::get_host() const {
        return impl_->host;
    }

    uint16_t FoxGloveComm::get_port() const {
        return impl_->port;
    }

    // image
    bool FoxGloveComm::create_image_channel(const std::string &topic) {
        if (!impl_ || !impl_->ready) {
            return false;
        }

        if (impl_->image_channels.find(topic) != impl_->image_channels.end()) {
            LOG_WARN(MODULE, "Image channel '{}' already exists", topic);
            return false;
        }

        if (impl_->float_channels.find(topic) != impl_->float_channels.end()) {
            LOG_WARN(MODULE, "Topic '{}' already exists as a float channel", topic);
            return false;
        }

        if (impl_->point_cloud_channels.find(topic) != impl_->point_cloud_channels.end()) {
            LOG_WARN(MODULE, "Topic '{}' already exists as a point cloud channel", topic);
            return false;
        }

        auto result = foxglove::messages::CompressedImageChannel::create(topic);

        if (!result.has_value()) {
            LOG_ERROR(MODULE, "Failed to create image channel '{}'", topic);
            return false;
        }

        impl_->image_channels.emplace(topic, std::move(result.value()));

        LOG_INFO(MODULE, "Created image channel '{}'", topic);
        return true;
    }

    bool FoxGloveComm::publish_image(const std::string &topic, const cv::Mat &image, uint64_t timestamp_ns,
                                     const std::string &frame_id) {
        if (!impl_ || !impl_->ready) {
            return false;
        }

        auto it = impl_->image_channels.find(topic);

        if (it == impl_->image_channels.end()) {
            LOG_ERROR(MODULE, "Image channel '{}' not found", topic);
            return false;
        }

        // OpenCV -> JPEG
        std::vector<uint8_t> jpeg_buffer;

        if (!cv::imencode(".jpg", image, jpeg_buffer)) {
            LOG_ERROR(MODULE, "Failed to encode image");
            return false;
        }

        foxglove::messages::CompressedImage msg;

        // frame id
        msg.frame_id = frame_id;

        // jpeg format
        msg.format = "jpeg";

        // uint8_t -> std::byte
        msg.data.resize(jpeg_buffer.size());

        std::memcpy(msg.data.data(), jpeg_buffer.data(), jpeg_buffer.size());

        // timestamp
        foxglove::messages::Timestamp stamp;

        stamp.sec = static_cast<int64_t>(timestamp_ns / 1000000000ULL);

        stamp.nsec = static_cast<uint32_t>(timestamp_ns % 1000000000ULL);

        msg.timestamp = stamp;

        // publish
        it->second.log(msg);

        return true;
    }

    // float
    bool FoxGloveComm::create_float_channel(const std::string &topic) {
        if (!impl_ || !impl_->ready) {
            return false;
        }

        if (impl_->float_channels.find(topic) != impl_->float_channels.end()) {
            LOG_WARN(MODULE, "Float channel '{}' already exists", topic);
            return false;
        }

        if (impl_->image_channels.find(topic) != impl_->image_channels.end()) {
            LOG_WARN(MODULE, "Topic '{}' already exists as an image channel", topic);
            return false;
        }

        if (impl_->point_cloud_channels.find(topic) != impl_->point_cloud_channels.end()) {
            LOG_WARN(MODULE, "Topic '{}' already exists as a point cloud channel", topic);
            return false;
        }

        foxglove::Schema schema{
            .name = "FloatValue", .encoding = "jsonschema",
            .data = reinterpret_cast<const std::byte *>(FLOAT_JSON_SCHEMA), .data_len = sizeof(FLOAT_JSON_SCHEMA) - 1
        };

        auto result = foxglove::RawChannel::create(topic, "json", schema);

        if (!result.has_value()) {
            LOG_ERROR(MODULE, "Failed to create float channel '{}'", topic);
            return false;
        }

        impl_->float_channels.emplace(topic, std::move(result.value()));

        LOG_INFO(MODULE, "Created float channel '{}'", topic);
        return true;
    }

    bool FoxGloveComm::publish_float(const std::string &topic, float value, uint64_t timestamp_ns) {
        if (!impl_ || !impl_->ready) {
            return false;
        }

        auto it = impl_->float_channels.find(topic);

        if (it == impl_->float_channels.end()) {
            LOG_ERROR(MODULE, "Float channel '{}' not found", topic);
            return false;
        }

        if (!std::isfinite(value)) {
            LOG_WARN(MODULE, "Skip non-finite float value on channel '{}'", topic);
            return false;
        }

        std::ostringstream ss;
        ss.imbue(std::locale::classic());
        ss << "{\"value\":" << value << "}";

        const std::string payload = ss.str();
        std::optional<uint64_t> log_time = std::nullopt;

        if (timestamp_ns != 0) {
            log_time = timestamp_ns;
        }

        const auto error = it->second.log(reinterpret_cast<const std::byte *>(payload.data()), payload.size(),
                                          log_time);

        if (error != foxglove::FoxgloveError::Ok) {
            LOG_ERROR(MODULE, "Failed to publish float channel '{}': {}", topic, foxglove::strerror(error));
            return false;
        }

        return true;
    }

    // point cloud
    bool FoxGloveComm::create_point_cloud_channel(const std::string &topic) {
        if (!impl_ || !impl_->ready) {
            return false;
        }

        if (impl_->point_cloud_channels.find(topic) != impl_->point_cloud_channels.end()) {
            LOG_WARN(MODULE, "Point cloud channel '{}' already exists", topic);
            return false;
        }

        if (impl_->image_channels.find(topic) != impl_->image_channels.end()) {
            LOG_WARN(MODULE, "Topic '{}' already exists as an image channel", topic);
            return false;
        }

        if (impl_->float_channels.find(topic) != impl_->float_channels.end()) {
            LOG_WARN(MODULE, "Topic '{}' already exists as a float channel", topic);
            return false;
        }

        auto result = foxglove::messages::PointCloudChannel::create(topic);

        if (!result.has_value()) {
            LOG_ERROR(MODULE, "Failed to create point cloud channel '{}'", topic);
            return false;
        }

        impl_->point_cloud_channels.emplace(topic, std::move(result.value()));

        LOG_INFO(MODULE, "Created point cloud channel '{}'", topic);
        return true;
    }

    bool FoxGloveComm::publish_point_cloud(const std::string &topic, const pcl::PointCloud<pcl::PointXYZ> &cloud,
                                           uint64_t timestamp_ns, const std::string &frame_id) {
        if (!impl_ || !impl_->ready) {
            return false;
        }

        auto it = impl_->point_cloud_channels.find(topic);

        if (it == impl_->point_cloud_channels.end()) {
            LOG_ERROR(MODULE, "Point cloud channel '{}' not found", topic);
            return false;
        }

        std::vector<float> xyz_data;
        xyz_data.reserve(cloud.points.size() * POINT_CLOUD_FIELD_COUNT);

        std::size_t skipped_count = 0;

        for (const auto &point: cloud.points) {
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
                ++skipped_count;
                continue;
            }

            xyz_data.emplace_back(point.x);
            xyz_data.emplace_back(point.y);
            xyz_data.emplace_back(point.z);
        }

        if (skipped_count > 0) {
            LOG_DEBUG(MODULE, "Skipped {} non-finite points on channel '{}'", skipped_count, topic);
        }

        foxglove::messages::PointCloud msg;
        msg.timestamp = make_timestamp(timestamp_ns);
        msg.frame_id = frame_id;
        msg.point_stride = POINT_CLOUD_POINT_STRIDE;
        msg.fields.emplace_back(make_float32_field("x", 0));
        msg.fields.emplace_back(make_float32_field("y", sizeof(float)));
        msg.fields.emplace_back(make_float32_field("z", 2 * sizeof(float)));

        msg.data.resize(xyz_data.size() * sizeof(float));

        if (!xyz_data.empty()) {
            std::memcpy(msg.data.data(), xyz_data.data(), msg.data.size());
        }

        it->second.log(msg);

        return true;
    }
} // namespace tools
