#pragma once

#include <unistd.h>
#include <iostream>
#ifndef __APPLE__
#include <linux/virtio_blk.h>
#endif
#include <sys/stat.h>
#include <unistd.h>
#include "Virtio.hpp"

class Blk : public Virtio {

  public:

    // input file as disk image
    Blk(bool readonly);

    ~Blk() override
    {
      if (fd_)
        close(fd_);
    }

    bool open_file(const std::string& filename);

  private:

    bool setup() final
    {
      if (not Virtio::setup())
        return false;

      if (not fd_)
        return false;

      // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
      config_ = reinterpret_cast<virtio_blk_config*>(device_cfg_);

      struct stat st{};
      fstat(fd_, &st);
      config_->capacity = st.st_size/512;
      return true;
    };

    void operator()(unsigned vq) final;

    int fd_ = -1;
    virtio_blk_config* config_ = nullptr;
};
