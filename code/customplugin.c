#include <gst/gst.h>
#include <gst/base/gstpushsrc.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>

#define CAMERA_DEVICE "/dev/video0"
#define RADAR_CFG "/dev/ttyACM0"
#define RADAR_DATA "/dev/ttyACM1"
#define READ_BUF_SIZE 64000

GST_DEBUG_CATEGORY_STATIC (gst_custom_src_debug);
#define GST_CAT_DEFAULT gst_custom_src_debug

typedef struct _GstCustomSrc {
  GstPushSrc parent;
  int camera_fd;
  int radar_cfg_fd;
  int radar_data_fd;
  std::vector char read_data_buffer;
} GstCustomSrc;

typedef struct _GstCustomSrcClass {
  GstPushSrcClass parent_class;
} GstCustomSrcClass;

G_DEFINE_TYPE (GstCustomSrc, gst_custom_src, GST_TYPE_PUSH_SRC);

static gboolean gst_custom_src_start (GstBaseSrc * src);
static gboolean gst_custom_src_stop (GstBaseSrc * src);
static GstFlowReturn gst_custom_src_create (GstPushSrc * src, GstBuffer ** buf);
static gboolean gst_custom_src_init_radar (GstBaseSrc * src);
static void gst_custom_src_close_radar (GstBaseSrc * src);

static void
gst_custom_src_class_init (GstCustomSrcClass * klass)
{
  GstElementClass *element_class = GST_ELEMENT_CLASS (klass);
  GstBaseSrcClass *base_src_class = GST_BASE_SRC_CLASS (klass);
  GstPushSrcClass *push_src_class = GST_PUSH_SRC_CLASS (klass);

  base_src_class->start = GST_DEBUG_FUNCPTR (gst_custom_src_start);
  base_src_class->stop = GST_DEBUG_FUNCPTR (gst_custom_src_stop);
  push_src_class->create = GST_DEBUG_FUNCPTR (gst_custom_src_create);

  GST_DEBUG_CATEGORY_INIT (gst_custom_src_debug, "customsrc", 0, "Custom Source");
}

static void
gst_custom_src_init (GstCustomSrc * src)
{
  src->camera_fd = -1;
  src->radar_cfg_fd = -1;
  src->radar_data_fd = -1;
}

static gboolean
gst_custom_src_start (GstBaseSrc * src)
{
  GstCustomSrc *custom_src = (GstCustomSrc *) src;
  if(!gst_custom_src_init_radar(custom_src))
  {//Initialisation of radar
    return FALSE;
  }

  custom_src->camera_fd = open (CAMERA_DEVICE, O_RDONLY); // Camera initialisation
  if (custom_src->camera_fd == -1) {
    GST_ERROR ("Failed to open camera device: %s", strerror (errno));
    return FALSE;
  }

  custom_src->radar_cfg_fd = open (RADAR_CFG, O_RDONLY);
  if (custom_src->radar_cfg_fd == -1) {
    GST_ERROR ("Failed to open radar device 1: %s", strerror (errno));
    close (custom_src->camera_fd);
    return FALSE;
  }

  custom_src->radar_data_fd = open (RADAR_DATA, O_RDONLY);
  if (custom_src->radar_data_fd == -1) {
    GST_ERROR ("Failed to open radar device 2: %s", strerror (errno));
    close (custom_src->camera_fd);
    close (custom_src->radar_cfg_fd);
    return FALSE;
  }

  return TRUE;
}

static void
gst_custom_src_close_radar (GstCustomSrc * src)
{
  if (src->radar_cfg_fd != -1) {
    close(src->radar_cfg_fd);
  }
  if (src->radar_data_fd != -1) {
    close(src->radar_data_fd);
  }
}

static gboolean
gst_custom_src_stop (GstBaseSrc * src)
{
  GstCustomSrc *custom_src = (GstCustomSrc *) src;

  if (custom_src->camera_fd != -1)
    close (custom_src->camera_fd);
  if (custom_src->radar_cfg_fd != -1)
    close (custom_src->radar_cfg_fd);
  if (custom_src->radar_data_fd != -1)
    close (custom_src->radar_data_fd);

  return TRUE;
}

static gboolean
gst_custom_src_init_radar (GstCustomSrc * src)
{
  struct termios tty;

  // Open radar config port
  src->radar_cfg_fd = open (RADAR_CFG, O_RDWR | O_NOCTTY | O_SYNC);
  if (src->radar_cfg_fd == -1) {
    GST_ERROR ("Failed to open radar config device: %s", strerror (errno));
    return FALSE;
  }

  if (tcgetattr(src->radar_cfg_fd, &tty) != 0) {
    GST_ERROR ("Error %i from tcgetattr: %s", errno, strerror(errno));
    return FALSE;
  }

  if (cfsetspeed(&tty, B115200) != 0) {
    GST_ERROR ("Error %i from cfsetspeed: %s", errno, strerror(errno));
    return FALSE;
  }

  if (tcsetattr(src->radar_cfg_fd, TCSANOW, &tty) != 0) {
    GST_ERROR ("Error %i from tcsetattr: %s", errno, strerror(errno));
    return FALSE;
  }

  // Open radar data port
  src->radar_data_fd = open (RADAR_DATA, O_RDWR | O_NOCTTY | O_SYNC);
  if (src->radar_data_fd == -1) {
    GST_ERROR ("Failed to open radar data device: %s", strerror (errno));
    close(src->radar_cfg_fd);
    return FALSE;
  }

  if (tcgetattr(src->radar_data_fd, &tty) != 0) {
    GST_ERROR ("Error %i from tcgetattr: %s", errno, strerror(errno));
    return FALSE;
  }

  if (cfsetspeed(&tty, B921600) != 0) {
    GST_ERROR ("Error %i from cfsetspeed: %s", errno, strerror(errno));
    return FALSE;
  }

  if (tcsetattr(src->radar_data_fd, TCSANOW, &tty) != 0) {
    GST_ERROR ("Error %i from tcsetattr: %s", errno, strerror(errno));
    return FALSE;
  }

  return TRUE;
}


static GstFlowReturn
gst_custom_src_create (GstPushSrc  src, GstBuffer * buf)
{
  GstCustomSrc custom_src = (GstCustomSrc ) src;
  GstBuffer *buffer;
  GstMapInfo map;
  char read_buf[READ_BUF_SIZE];
  int read_size;

  // Read from radar data port
  read_size = read(custom_src->radar_data_fd, read_buf, sizeof(read_buf));
  if (read_size == -1) {
    GST_ERROR ("Failed to read from radar data device: %s", strerror (errno));
    return GST_FLOW_ERROR;
  }

  // Process radar data (example)
  std::vector<char> &data = custom_src->radar_data_buffer;
  data.insert(data.end(), read_buf, read_buf + read_size);

  // Create buffer to hold radar data
  buffer = gst_buffer_new_allocate(NULL, read_size, NULL);
  gst_buffer_map(buffer, &map, GST_MAP_WRITE);
  memcpy(map.data, read_buf, read_size);
  gst_buffer_unmap(buffer, &map);

  *buf = buffer;
  return GST_FLOW_OK;
}

static gboolean
plugin_init (GstPlugin * plugin)
{
  return gst_element_register (plugin, "customsrc", GST_RANK_NONE, GST_TYPE_CUSTOM_SRC);
}

GST_PLUGIN_DEFINE (
    GST_VERSION_MAJOR,
    GST_VERSION_MINOR,
    customsrc,
    "Custom Source Plugin",
    plugin_init,
    VERSION,
    "LGPL",
    "GStreamer",
    "http://gstreamer.net/"
)

