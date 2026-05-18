//
// Created by Interweave on 2026/4/14.
//

#include "foxglove_comm.hpp"

#include <optional>
#include <utility>

#include "foxglove/server.hpp"
#include "foxglove/channel.hpp"
#include "foxglove/messages.hpp"
#include "tools/logger.hpp"


namespace tools {
    struct FoxGloveComm::Impl {
        std::string host;
        uint16_t port = 0;
        bool ready = false;

        std::optional<foxglove::WebSocketServer> server;

        LoggerConfig cfg{
            .level = LogLevel::Debug,
            .enable_console = true,
            .enable_file = false
        };

        std::unordered_map<std::string, foxglove::messages::CompressedImageChannel> image_channels;

        Impl(const std::string &h, uint16_t p) : host(h), port(p) {
        }

        bool create_server() {
            ready = false;
            server.reset();

            auto result = foxglove::WebSocketServer::create(
                foxglove::WebSocketServerOptions{
                    .host = host,
                    .port = port
                }
            );

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
    bool FoxGloveComm::create_image_channel(
        const std::string &topic) {
        if (!impl_ || !impl_->ready) {
            return false;
        }

        if (impl_->image_channels.find(topic) !=
            impl_->image_channels.end()) {
            LOG_WARN(MODULE, "Image channel '{}' already exists", topic);
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

    bool FoxGloveComm::publish_image(const std::string &topic, const cv::Mat &image, uint64_t timestamp_ns, const std::string &frame_id) {
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

        if (!cv::imencode(".jpg", image,jpeg_buffer)) {
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

        std::memcpy(
            msg.data.data(),
            jpeg_buffer.data(),
            jpeg_buffer.size());

        // timestamp
        foxglove::messages::Timestamp stamp;

        stamp.sec = static_cast<int64_t>(timestamp_ns / 1000000000ULL);

        stamp.nsec =static_cast<uint32_t>(timestamp_ns % 1000000000ULL);

        msg.timestamp = stamp;

        // publish
        it->second.log(msg);

        return true;
    }
} // namespace tools
