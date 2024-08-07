GST_PLUGIN_NAME = customsrc
SOURCES = customplugin.c

CFLAGS = $(shell pkg-config --cflags gstreamer-1.0 gstreamer-base-1.0)
LIBS = $(shell pkg-config --libs gstreamer-1.0 gstreamer-base-1.0)

$(GST_PLUGIN_NAME).so: $(SOURCES)
    $(CC) -shared -o $@ $^ $(CFLAGS) $(LIBS)

clean:
    rm -f $(GST_PLUGIN_NAME).so