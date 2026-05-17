#include "../include/lane_pipe_writer.h"

#include <cerrno>
#include <csignal>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

namespace {
    constexpr const char *kLanePipePath = "/tmp/pacerbot_lane.pipe";
    int g_pipe_fd                       = -1;
    bool g_sigpipeIgnored               = false;

    bool ensurePipeExists()
    {
        if (mkfifo(kLanePipePath, 0666) == 0) {
            return true;
        }

        return errno == EEXIST;
    }

    bool ignoreSigpipe()
    {
        if (g_sigpipeIgnored) {
            return true;
        }

        if (std::signal(SIGPIPE, SIG_IGN) == SIG_ERR) {
            return false;
        }

        g_sigpipeIgnored = true;
        return true;
    }

    void closePipe()
    {
        if (g_pipe_fd >= 0) {
            close(g_pipe_fd);
            g_pipe_fd = -1;
        }
    }

    bool openPipeIfNeeded()
    {
        if (g_pipe_fd >= 0) {
            return true;
        }

        if (!ensurePipeExists() || !ignoreSigpipe()) {
            return false;
        }

        g_pipe_fd = open(kLanePipePath, O_WRONLY | O_NONBLOCK);
        if (g_pipe_fd < 0) {
            return false;
        }

        return true;
    }
} // namespace

bool initializeLanePipeWriter() { return openPipeIfNeeded(); }

bool sendLaneInput(bool valid, float steering_error)
{
    if (!openPipeIfNeeded()) {
        return false;
    }

    const LaneInput input {valid, steering_error};
    const ssize_t bytesWritten = write(g_pipe_fd, &input, sizeof(input));

    if (bytesWritten == static_cast<ssize_t>(sizeof(input))) {
        return true;
    }

    if (bytesWritten < 0
        && (errno == EPIPE || errno == ENXIO || errno == EAGAIN || errno == EWOULDBLOCK
            || errno == EINTR)) {
        if (errno == EPIPE || errno == ENXIO) {
            closePipe();
        }
        return false;
    }

    closePipe();
    return false;
}