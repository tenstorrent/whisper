#include <iostream>
#include <sys/mman.h>
#include <cassert>
#include <thread>
#include <chrono>
#include <fstream>
#include <cstring>
#include "RemoteFrameBuffer.hpp"
#include <rfb/rfb.h>

using namespace WdRiscv;

// Don't change these
enum {
RFB_BITS_PER_SAMPLE = 8,
RFB_SAMPLES_PER_PIXEL = 3
};

// Time between RFB updates
enum {
RFB_FRAME_TIME_US =  100000
};

RemoteFrameBuffer::RemoteFrameBuffer(uint64_t addr, uint64_t width, uint64_t height, uint64_t bytes_per_pixel, int port)
  : IoDevice("frame_buffer", addr, width*height*bytes_per_pixel), width_(width), height_(height), bytes_per_pixel_(bytes_per_pixel), port_(port) {

  // TODO: either hard code it to be 4 or support 1,2,4 bpp
  assert(bytes_per_pixel == 4 && "bytes per pixel must be 4");

  // each value in the frame buffer represents a pixel
  frame_buffer_.resize(size() >> 2 /* size in bytes */, 0);

  auto func = [this]() { this->vncServerLoop(); };
  displayThread_ = std::thread(func);
}

RemoteFrameBuffer::~RemoteFrameBuffer()
{
  terminate_ = true;
  if (displayThread_.joinable())
    displayThread_.join();
}

void
RemoteFrameBuffer::vncServerLoop()
{
#ifdef REMOTE_FRAME_BUFFER

  // libvncserver needs command line args passed to it
  int rfbArgc = 1;
  char programName[] = "whisper";
  char *rfbArgv[] = {programName, nullptr};

  rfbScreenInfoPtr rfbScreen = rfbGetScreen(&rfbArgc, rfbArgv, width_, height_, RFB_BITS_PER_SAMPLE, RFB_SAMPLES_PER_PIXEL, bytes_per_pixel_);
  if (!rfbScreen)
    return;
  rfbScreen->desktopName = "Whisper VNC";
  rfbScreen->frameBuffer = (char*) frame_buffer_.data();
  rfbScreen->alwaysShared = TRUE;
  rfbScreen->port = port_;

  rfbInitServer(rfbScreen);

  while(rfbIsActive(rfbScreen) && !terminate_)
  {
    rfbProcessEvents(rfbScreen, RFB_FRAME_TIME_US);
    if (frame_buffer_updated_) {
        rfbMarkRectAsModified(rfbScreen, 0, 0, width_, height_);
        frame_buffer_updated_ = false;
    }
  }

  rfbScreenCleanup(rfbScreen);

#endif

}

uint32_t
RemoteFrameBuffer::read(uint64_t addr)
{
  uint64_t offset = addr - address();

  if (offset >= size())
    return 0;

  return frame_buffer_.at(offset/4);
}

void
RemoteFrameBuffer::write(uint64_t addr, uint32_t value)
{
  uint64_t offset = addr - address();

  assert(offset < size() && "RemoteFrameBuffer: Writing outside of buffer range");
  frame_buffer_.at(offset/4) = value;
  frame_buffer_updated_ = true;
}

void
RemoteFrameBuffer::enable()
{
}

void
RemoteFrameBuffer::disable()
{
}

bool
RemoteFrameBuffer::saveSnapshot(const std::string& filename) const
{
  std::ofstream ofs(filename, std::ios::binary);
  if (!ofs)
    {
      std::cerr << "Error: failed to open snapshot file for writing: " << filename << "\n";
      return false;
    }

  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
  ofs.write(reinterpret_cast<const char*>(frame_buffer_.data()), long(size()));

  if (!ofs)
    {
      std::cerr << "Error: failed to write frame buffer data to: " << filename << "\n";
      return false;
    }

  return true;
}

bool
RemoteFrameBuffer::loadSnapshot(const std::string& filename)
{
  std::ifstream ifs(filename, std::ios::binary);
  if (!ifs)
    {
      std::cerr << "Warning: failed to open snapshot file " << filename << "\n";
      return false;
    }

  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
  ifs.read(reinterpret_cast<char*>(frame_buffer_.data()), long(size()));

  if (!ifs)
    {
      std::cerr << "Error: failed to read frame buffer data from " << filename << "\n";
      return false;
    }

  frame_buffer_updated_ = true;
  return true;
}
